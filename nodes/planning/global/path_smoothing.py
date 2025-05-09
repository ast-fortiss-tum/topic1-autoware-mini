#!/usr/bin/env python3

import time
import rospy
import numpy as np
from autoware_msgs.msg import Lane, Waypoint
from std_msgs.msg import Float64MultiArray 
from helpers.geometry import get_orientation_from_heading


class PathSmoothing:

    def __init__(self):

        # Parameters
        self.waypoint_interval = rospy.get_param("waypoint_interval")
        self.adjust_speeds_in_curves = rospy.get_param("~adjust_speeds_in_curves")
        self.adjust_speeds_using_deceleration = rospy.get_param("~adjust_speeds_using_deceleration")
        self.adjust_endpoint_speed_to_zero = rospy.get_param("~adjust_endpoint_speed_to_zero")
        self.default_deceleration = rospy.get_param("default_deceleration")
        self.speed_averaging_window = rospy.get_param("~speed_averaging_window")
        self.radius_calc_neighbour_index = rospy.get_param("~radius_calc_neighbour_index")
        self.lateral_acceleration_limit = rospy.get_param("~lateral_acceleration_limit")
        self.output_debug_info = rospy.get_param("~output_debug_info")
        self.exec_time_pub = rospy.Publisher(
                              f"/{rospy.get_name()}/exec_time_with_stamp",
                              Float64MultiArray,
                              queue_size=10
                             ) 
        # Publishers
        self.smoothed_path_pub = rospy.Publisher('smoothed_path', Lane, queue_size=10, latch=True, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('global_path', Lane, self.global_path_callback, queue_size=None, tcp_nodelay=True)


    def global_path_callback(self, msg):
        start_time = time.time()
        try:
         stampe = msg.header.stamp.to_sec()
         if stampe == 0.0:
          raise ValueError("Zero stamp")
        except:
         stampe = rospy.get_rostime().to_sec()
        if len(msg.waypoints) == 0:
            # create marker_array to delete all visualization markers
            self.publish_smoothed_path(np.array([]), msg.header.frame_id)
            return

        # extract all waypoint attributes with one loop
        waypoints_array = np.array([(
                wp.pose.pose.position.x,
                wp.pose.pose.position.y,
                wp.pose.pose.position.z,
                wp.wpstate.steering_state,
                wp.twist.twist.linear.x,
                wp.dtlane.lw,
                wp.dtlane.rw
            ) for wp in msg.waypoints])

        smoothed_path_array = self.smooth_global_path(waypoints_array)
        exec_duration = time.time() - start_time
        timing_msg = Float64MultiArray()
        timing_msg.data = [stampe, exec_duration]
        self.exec_time_pub.publish(timing_msg)

        rospy.loginfo(f"[{rospy.get_name()}] Exec time: {exec_duration:.6f}s | Stamp: {stampe:.3f}")
        self.publish_smoothed_path(smoothed_path_array, msg.header.frame_id)
 

    def smooth_global_path(self, waypoints_array):

        # extract into arrays
        xy_path = waypoints_array[:,:2]
        x_path = waypoints_array[:,0]
        y_path = waypoints_array[:,1]
        z_path = waypoints_array[:,2]
        blinker = waypoints_array[:,3]
        speed = waypoints_array[:,4]
        lw = waypoints_array[:,5]
        rw = waypoints_array[:,6]

        # distance differeneces between points
        distances = np.cumsum(np.sqrt(np.sum(np.diff(xy_path, axis=0)**2, axis=1)))
        # add 0 to the beginning of the array
        distances = np.insert(distances, 0, 0)
        # create new distances at fixed intervals
        new_distances = np.linspace(0, distances[-1], num=int(distances[-1] / self.waypoint_interval))

        #### INTERPOLATE  ####

        # interpolate x_new, y_new, z_new
        x_new = np.interp(new_distances, distances, x_path)
        y_new = np.interp(new_distances, distances, y_path)
        z_new = np.interp(new_distances, distances, z_path)

        # Blinkers
        blinker_left = np.rint(np.interp(new_distances, distances, (blinker == 1).astype(np.uint8)))
        blinker_right = np.rint(np.interp(new_distances, distances, (blinker == 2).astype(np.uint8)))
        # combine arrays into one so that left = 1, right = 2, straight = 3
        blinker_new = np.maximum(blinker_left, blinker_right * 2)
        blinker_new[blinker_new == 0] = 3

        # Yaw - calculate yaw angle for path and add last yaw angle to the end of the array
        yaw = np.arctan2(np.diff(y_new), np.diff(x_new))
        yaw = np.append(yaw, yaw[-1])

        # lw and rw
        lw_new = np.interp(new_distances, distances, lw)
        rw_new = np.interp(new_distances, distances, rw)

        # Speed
        speed_interpolated = np.interp(new_distances, distances, speed)
        speed_new = speed_interpolated

        if self.adjust_speeds_in_curves and len(speed_new) >= 3:
            # Calculate speed limit based on lateral acceleration limit
            radius = calculate_radius_step_n_triangle_equation(x_new, y_new, self.radius_calc_neighbour_index)
            speed_radius = np.sqrt(self.lateral_acceleration_limit * radius)
            speed_new = np.fmin(speed_interpolated, speed_radius)

        # loop over array backwards and forwards to adjust speeds using the deceleration limit
        if self.adjust_speeds_using_deceleration:
            accel_constant = 2 * self.default_deceleration * self.waypoint_interval
            # backward loop
            for i in range(len(speed_new) - 2, 0, -1):
                speed_new[i] = min(speed_new[i], np.sqrt(speed_new[i + 1]**2 + accel_constant))
            # forward loop
            for i in range(1, len(speed_new) ):
                speed_new[i] = min(speed_new[i], np.sqrt(speed_new[i - 1]**2 + accel_constant))

        if self.speed_averaging_window > 1 and len(speed_new) >= self.speed_averaging_window:
            # average array values using window size of n
            speed_new = np.convolve(speed_new, np.ones((self.speed_averaging_window,))/self.speed_averaging_window, mode='same')
            # replace n/2 values at the beginning and end of the array with the n+1 and n-1 values respectively
            speed_new[:int(self.speed_averaging_window/2)] = speed_new[int(self.speed_averaging_window/2)]
            speed_new[-int(self.speed_averaging_window/2):] = speed_new[-int(self.speed_averaging_window/2)-1]

        if self.adjust_endpoint_speed_to_zero:
            # set last point speed to zero
            speed_new[-1] = 0

            # adjust speed graphs using deceleartion limit and waypoint interval
            accel_constant = 2 * self.default_deceleration * self.waypoint_interval
            # backward loop - end point
            for i in range(len(speed_new) - 2, 0, -1):
                adjusted_speed = np.sqrt(speed_new[i + 1]**2 + accel_constant)
                if adjusted_speed > speed_new[i]:
                    break
                speed_new[i] = adjusted_speed

        if self.output_debug_info:
            debug_plots_path_smoothing(x_path, y_path, z_path, blinker, x_new, y_new, z_new, blinker_new, distances, new_distances, speed, speed_new)

        # Stack
        smoothed_path_array = np.stack((x_new, y_new, z_new, blinker_new, speed_new, lw_new, rw_new, yaw), axis=1)

        return smoothed_path_array

    def publish_smoothed_path(self, smoothed_path, output_frame):
        # create lane message
        lane = Lane()
        lane.header.frame_id = output_frame
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = [self.create_waypoint(*wp) for wp in smoothed_path]

        self.smoothed_path_pub.publish(lane)

    def create_waypoint(self, x, y, z, blinker, speed, lw, rw, yaw):
        # create waypoint
        waypoint = Waypoint()
        waypoint.pose.pose.position.x = x
        waypoint.pose.pose.position.y = y
        waypoint.pose.pose.position.z = z
        waypoint.wpstate.steering_state = int(blinker)
        waypoint.twist.twist.linear.x = speed
        waypoint.pose.pose.orientation = get_orientation_from_heading(yaw)
        waypoint.dtlane.lw = lw
        waypoint.dtlane.rw = rw

        return waypoint

    def run(self):
        rospy.spin()

def calculate_radius_step_n_triangle_equation(x, y, n):

    # equation derived from:
    # https://en.wikipedia.org/wiki/Circumscribed_circle#Other_properties
    # diameter = a*b*c / 2*area

    # adjust n for very short paths
    if len(x) < 2 * n + 1:
        n = int((len(x) - 1) / 2)

    # find the lengths of the 3 edges for the triangle
    a = np.sqrt((x[n:-n] - x[:-2*n])**2 + (y[n:-n] - y[:-2*n])**2)
    b = np.sqrt((x[:-2*n] - x[2*n:])**2 + (y[:-2*n] - y[2*n:])**2)
    c = np.sqrt((x[2*n:] - x[n:-n])**2 + (y[2*n:] - y[n:-n])**2)

    # calculate the area of the triangle
    s = (a + b + c) / 2
    # TODO: added np.abs because sometimes negative values are there, check why
    area = np.sqrt(np.abs(s * (s - a) * (s - b) * (s - c)))
    area = np.maximum(area, 0.0000000001)

    # calculate the radius of the circle
    radius = (a * b * c) / (4 * area)

    # append n values to the beginning of the array with the first value
    radius = np.append(np.repeat(radius[0], n), radius)
    # append n values to the end of the array with the last value
    radius = np.append(radius, np.repeat(radius[-1], n))

    return radius

def debug_plots_path_smoothing(x_path, y_path, z_path, blinker, x_new, y_new, z_new, blinker_new, distances, new_distances, speed, speed_new):

    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(x_path,y_path, color = 'blue')
    ax.scatter(x_new, y_new, color = 'red', marker = 'x', alpha = 0.5, label = 'interpolated')
    plt.legend()
    plt.show()

    # new plot for heights 
    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(new_distances, z_new, color = 'red', marker = 'x', alpha = 0.5, label = 'height interpolated')
    ax.plot(new_distances, z_new, color = 'red', alpha = 0.5, label = 'height interpolated')
    ax.scatter(distances, z_path, color = 'blue', alpha = 0.5, label = 'height old')
    plt.legend()
    plt.show()

    # new plot for blinkers
    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(new_distances, blinker_new, color = 'red', marker = 'x', alpha = 0.5, label = 'blinker interpolated')
    ax.plot(new_distances, blinker_new, color = 'red', alpha = 0.5, label = 'blinker interpolated')
    ax.scatter(distances, blinker, color = 'blue', alpha = 0.5, label = 'blinker old')
    plt.legend()
    plt.show()

    # new plot for speed
    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(new_distances, speed_new * 3.6, color = 'red', marker = 'x', alpha = 0.5, label = 'speed interpolated')
    ax.plot(new_distances, speed_new * 3.6, color = 'red', alpha = 0.5, label = 'speed interpolated')
    ax.scatter(distances, speed * 3.6, color = 'blue', alpha = 0.5, label = 'speed old')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    rospy.init_node('path_smoothing')
    node = PathSmoothing()
    node.run()