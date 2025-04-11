#!/usr/bin/env python3
#
# Copyright (c) 2023 Autonomous Driving Lab (ADL), University of Tartu.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
ground truth detections. Publishes the following topics:
    receive :derived_object_msgs::ObjectArray and publishes autoware_msgs::DetectedObjectArray
"""
import time
import rospy

from std_msgs.msg import ColorRGBA,Float64MultiArray
from geometry_msgs.msg import PolygonStamped, Point
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from derived_object_msgs.msg import ObjectArray, Object
from localization.SimulationToUTMTransformer import SimulationToUTMTransformer
from helpers.detection import create_hull

CLASS_ID_TO_LABEL = {
    Object.CLASSIFICATION_UNKNOWN: 'unknown',
    Object.CLASSIFICATION_UNKNOWN_SMALL: 'unknown_small',
    Object.CLASSIFICATION_UNKNOWN_MEDIUM: 'unknown_medium',
    Object.CLASSIFICATION_UNKNOWN_BIG: 'unknown_big',
    Object.CLASSIFICATION_PEDESTRIAN: 'pedestrian',
    Object.CLASSIFICATION_BIKE: 'bike',
    Object.CLASSIFICATION_CAR: 'car',
    Object.CLASSIFICATION_TRUCK: 'truck',
    Object.CLASSIFICATION_MOTORCYCLE: 'motorcycle',
    Object.CLASSIFICATION_OTHER_VEHICLE: 'other_vehicle',
    Object.CLASSIFICATION_BARRIER: 'barrier',
    Object.CLASSIFICATION_SIGN: 'sign'
}

YELLOW80P = ColorRGBA(1.0, 1.0, 0.0, 0.8)

class CarlaDetector:

    def __init__(self):

        # Node parameters
        self.use_transformer = rospy.get_param("/carla_localization/use_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")
        self.output_frame = rospy.get_param("/detection/output_frame")
        self.exec_time_pub = rospy.Publisher(
                              f"/{rospy.get_name()}/exec_time_with_stamp",
                              Float64MultiArray,
                              queue_size=10
                             )  
        # Internal parameters
        self.sim2utm_transformer = SimulationToUTMTransformer(use_custom_origin=use_custom_origin,
                                                              origin_lat=utm_origin_lat,
                                                              origin_lon=utm_origin_lon)

        # Publishers
        self.detected_objects_pub = rospy.Publisher(
            'detected_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/carla/ego_vehicle/objects',
                         ObjectArray, self.carla_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

    def carla_objects_callback(self, data):
        """
        callback for carla objects
        """
        start_time = time.time().to_sec()
        try:
         stampe = data.header.stamp.to_sec()
         if stampe == 0.0:
          raise ValueError("Zero stamp")
        except:
         stampe = rospy.get_rostime().to_sec()  
        objects_msg = DetectedObjectArray()
        objects_msg.header = data.header

        for obj in data.objects:

            object_msg = DetectedObject()
            object_msg.header = obj.header
            object_msg.id = obj.id
            object_msg.label = CLASS_ID_TO_LABEL[obj.classification] 
            object_msg.color = YELLOW80P
            object_msg.score = 1
            object_msg.valid = True
            object_msg.space_frame = self.output_frame
            object_msg.pose = obj.pose

            if self.use_transformer:
                object_msg.pose = self.sim2utm_transformer.transform_pose(object_msg.pose)

            object_msg.dimensions.x = obj.shape.dimensions[0]
            object_msg.dimensions.y = obj.shape.dimensions[1]
            object_msg.dimensions.z = obj.shape.dimensions[2]
            object_msg.velocity = obj.twist
            object_msg.acceleration = obj.accel
            object_msg.convex_hull = create_hull(object_msg, self.output_frame, object_msg.header.stamp)
            object_msg.pose_reliable = True
            object_msg.velocity_reliable = True
            object_msg.acceleration_reliable = True
            object_msg.valid = True

            objects_msg.objects.append(object_msg)
        exec_duration = time.time()- start_time
        timing_msg = Float64MultiArray()
        timing_msg.data = [stampe, exec_duration]
        self.exec_time_pub.publish(timing_msg)

        rospy.loginfo(f"[{rospy.get_name()}] Exec time: {exec_duration:.6f}s | Stamp: {stampe:.3f}")
        # Publish converted detected objects
        self.detected_objects_pub.publish(objects_msg)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('carla_detector', log_level=rospy.INFO)
    node = CarlaDetector()
    node.run()
