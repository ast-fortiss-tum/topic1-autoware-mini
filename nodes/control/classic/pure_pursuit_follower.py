#!/usr/bin/env python3

import rospy
import math
import message_filters
import threading
import traceback
import time 
from shapely.geometry import Point as ShapelyPoint
from shapely import distance

from helpers.geometry import get_heading_from_orientation, normalize_heading_error, get_heading_between_two_points
from helpers.path import Path

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped, Point
from std_msgs.msg import ColorRGBA, Float32MultiArray,Float64MultiArray
from autoware_msgs.msg import Lane, VehicleCmd


class PurePursuitFollower:
    def __init__(self):

        # Parameters
        self.lookahead_time = rospy.get_param("~lookahead_time")
        self.min_lookahead_distance = rospy.get_param("~min_lookahead_distance")
        self.wheel_base = rospy.get_param("/vehicle/wheel_base")
        self.heading_angle_limit = rospy.get_param("heading_angle_limit")
        self.lateral_error_limit = rospy.get_param("lateral_error_limit")
        self.blinker_lookahead_time = rospy.get_param("blinker_lookahead_time")
        self.blinker_min_lookahead_distance = rospy.get_param("blinker_min_lookahead_distance")
        self.current_pose_to_car_front = rospy.get_param("/planning/current_pose_to_car_front")
        self.publish_debug_info = rospy.get_param("~publish_debug_info")
        self.default_acceleration = rospy.get_param("/planning/default_acceleration")
        self.default_deceleration = rospy.get_param("/planning/default_deceleration")
        self.max_deceleration = rospy.get_param("/planning/max_deceleration")
        self.stopping_speed_limit = rospy.get_param("/planning/stopping_speed_limit")
        self.simulate_cmd_delay = rospy.get_param("~simulate_cmd_delay")

        # Variables - init
        self.path = None
        self.closest_object_velocity = 0.0
        self.stopping_point_distance = 0.0
        self.lock = threading.Lock()

        # Publishers
        self.vehicle_command_pub = rospy.Publisher('vehicle_cmd', VehicleCmd, queue_size=1, tcp_nodelay=True)
        if self.publish_debug_info:
            self.pure_pursuit_markers_pub = rospy.Publisher('follower_markers', MarkerArray, queue_size=1, tcp_nodelay=True)
            self.follower_debug_pub = rospy.Publisher('follower_debug', Float32MultiArray, queue_size=1, tcp_nodelay=True)
        self.exec_time_pub = rospy.Publisher(
                              f"/{rospy.get_name()}/exec_time_with_stamp",
                              Float64MultiArray,
                              queue_size=10
                              ) 
        # Subscribers
        rospy.Subscriber('/planning/local_path', Lane, self.path_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        current_pose_sub = message_filters.Subscriber('/localization/current_pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        current_velocity_sub = message_filters.Subscriber('/localization/current_velocity', TwistStamped, queue_size=1, tcp_nodelay=True)
        ts = message_filters.ApproximateTimeSynchronizer([current_pose_sub, current_velocity_sub], queue_size=2, slop=0.02)
        ts.registerCallback(self.current_status_callback)

        # output information to console
        rospy.loginfo("%s - initialized", rospy.get_name())

    def path_callback(self, path_msg):

        if len(path_msg.waypoints) == 0:
            # if path is cancelled and empty waypoints received
            path = None
            closest_object_velocity = 0.0
            stopping_point_distance = 0.0
        else:
            path = Path(path_msg.waypoints, velocities=True, blinkers=True)
            closest_object_velocity = path_msg.closest_object_velocity
            stopping_point_distance = path_msg.cost

        with self.lock:
            self.path = path
            self.closest_object_velocity = closest_object_velocity
            self.stopping_point_distance = stopping_point_distance

    def current_status_callback(self, current_pose_msg, current_velocity_msg):
        start_time1 = time.time()
        try:
         stampe = current_pose_msg.header.stamp.to_sec()
         if stampe == 0.0:
          raise ValueError("Zero stamp")
        except:
         stampe = rospy.get_rostime().to_sec()
        try:
            if self.publish_debug_info:
                start_time = rospy.get_time()

            with self.lock:
                path = self.path
                closest_object_velocity = self.closest_object_velocity
                stopping_point_distance = self.stopping_point_distance

            stamp = current_pose_msg.header.stamp

            if path is None:
                self.publish_vehicle_command(stamp)
                rospy.logwarn_throttle(30, "%s - no waypoints, stopping!", rospy.get_name())
                return

            current_position = current_pose_msg.pose.position
            current_heading = get_heading_from_orientation(current_pose_msg.pose.orientation)
            current_velocity = current_velocity_msg.twist.linear.x

            cross_track_error = path.get_cross_track_error(current_position)

            # simulate delay in vehicle command. Project car into future location and use it to calculate current steering command
            if self.simulate_cmd_delay > 0.0:
                current_position.x += math.cos(current_heading) * current_velocity * self.simulate_cmd_delay
                current_position.y += math.sin(current_heading) * current_velocity * self.simulate_cmd_delay

            current_position_shapely = ShapelyPoint(current_position.x, current_position.y)
            ego_distance_from_path_start = path.linestring.project(current_position_shapely)

            # if "waypoint planner" is used and no global and local planner involved
            if ego_distance_from_path_start >= path.linestring.length:
                self.publish_vehicle_command(stamp)
                rospy.logwarn_throttle(10, "%s - end of path reached", rospy.get_name())
                return

            lookahead_distance = current_velocity * self.lookahead_time
            lookahead_distance = max(lookahead_distance, self.min_lookahead_distance)

            # find lookahead_point on path
            lookahead_point = path.linestring.interpolate(ego_distance_from_path_start + lookahead_distance)
            ego_distance_to_lookahead_point = distance(current_position_shapely, lookahead_point)

            # heading from quaternion in current pose orientation
            lookahead_heading = get_heading_between_two_points(current_position, lookahead_point)
            heading_differenece = lookahead_heading - current_heading

            heading_angle_difference = normalize_heading_error(heading_differenece)

            if abs(cross_track_error) > self.lateral_error_limit or abs(math.degrees(heading_angle_difference)) > self.heading_angle_limit:
                # stop vehicle if cross track error or heading angle difference is over limit
                self.publish_vehicle_command(stamp)
                rospy.logerr_throttle(10, "%s - lateral error or heading angle difference over limit", rospy.get_name())
                return

            # calculate curvature and steering angle
            curvature = 2 * math.sin(heading_differenece) / ego_distance_to_lookahead_point
            steering_angle = math.atan(self.wheel_base * curvature)

            # find velocity at current position
            target_velocity = path.get_velocity_at_distance(ego_distance_from_path_start)
            if target_velocity < self.stopping_speed_limit:
                target_velocity = 0.0

            # if decelerating because of obstacle then calculate necessary deceleration
            emergency = 0
            if stopping_point_distance > 0.0 and target_velocity < current_velocity:
                # calculate distance from car front to stopping point
                car_front_to_stopping_point_distance = stopping_point_distance - ego_distance_from_path_start - self.current_pose_to_car_front
                if car_front_to_stopping_point_distance > 0:
                    # always allow minimum deceleration, to be able to adapt to map speeds
                    acceleration = min(0.5 * (closest_object_velocity**2 - current_velocity**2) / car_front_to_stopping_point_distance, -self.default_deceleration)
                else:
                    # emergency braking - car front over the stopping point
                    acceleration = -self.max_deceleration
                    emergency = 1
            else:
                # use the default acceleration/deceleration limits
                if target_velocity > current_velocity:
                    acceleration = self.default_acceleration
                else:
                    acceleration = -self.default_deceleration

            blinker_lookahead_distance = max(self.blinker_min_lookahead_distance, self.blinker_lookahead_time * current_velocity)
            left_blinker, right_blinker = path.get_blinker_state_with_lookahead(ego_distance_from_path_start, blinker_lookahead_distance)

            # Publish
            self.publish_vehicle_command(stamp, steering_angle, target_velocity, acceleration, left_blinker, right_blinker, emergency)
            if self.publish_debug_info:
                # convert lookahead_point from shpely 2d point to geometry_msg/Point
                lookahead_point = Point(x=lookahead_point.x, y=lookahead_point.y, z=current_pose_msg.pose.position.z)
                self.publish_pure_pursuit_markers(current_pose_msg.header, current_pose_msg.pose.position, lookahead_point, heading_angle_difference)
                self.follower_debug_pub.publish(Float32MultiArray(data=[(rospy.get_time() - start_time), current_heading, lookahead_heading, heading_angle_difference, cross_track_error, target_velocity]))
            exec_duration = time.time() - start_time1
            timing_msg = Float64MultiArray()
            timing_msg.data = [stampe, exec_duration]
            self.exec_time_pub.publish(timing_msg)

            rospy.loginfo(f"[{rospy.get_name()}] Exec time: {exec_duration:.6f}s | Stamp: {stampe:.3f}")
        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())


    def publish_vehicle_command(self, stamp, steering_angle=0.0, target_velocity=0.0, acceleration=0.0, left_blinker=0, right_blinker=0, emergency=0):
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.header.stamp = stamp
        vehicle_cmd.header.frame_id =  "base_link"
        # blinkers
        vehicle_cmd.lamp_cmd.l = left_blinker
        vehicle_cmd.lamp_cmd.r = right_blinker
        # velocity and steering
        vehicle_cmd.ctrl_cmd.linear_velocity = target_velocity
        vehicle_cmd.ctrl_cmd.linear_acceleration = acceleration
        vehicle_cmd.ctrl_cmd.steering_angle = steering_angle
        vehicle_cmd.emergency = emergency
        self.vehicle_command_pub.publish(vehicle_cmd)


    def publish_pure_pursuit_markers(self, header, current_position, lookahead_point, heading_error):

        marker_array = MarkerArray()

        # draws a line between current pose and lookahead point
        marker = Marker(header=header)
        marker.ns = "Lookahead line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.color = ColorRGBA(1.0, 0.0, 1.0, 1.0)
        marker.points = ([current_position, lookahead_point])
        marker_array.markers.append(marker)

        # label heading_error
        average_pose = Pose()
        average_pose.position.x = (current_position.x + lookahead_point.x) / 2
        average_pose.position.y = (current_position.y + lookahead_point.y) / 2
        average_pose.position.z = (current_position.z + lookahead_point.z) / 2
        average_pose.orientation.w = 1.0

        marker_text = Marker(header=header)
        marker_text.ns = "Heading error"
        marker_text.id = 1
        marker_text.type = Marker.TEXT_VIEW_FACING
        marker_text.action = Marker.ADD
        marker_text.pose = average_pose
        marker_text.scale.z = 0.6
        marker_text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        marker_text.text = str(round(math.degrees(heading_error),1))
        marker_array.markers.append(marker_text)

        self.pure_pursuit_markers_pub.publish(marker_array)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower', log_level=rospy.INFO)
    node = PurePursuitFollower()
    node.run()