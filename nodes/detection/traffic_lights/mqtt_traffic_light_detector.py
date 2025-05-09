#!/usr/bin/env python3

import rospy
import json
import time
import traceback

import paho.mqtt.client as paho

from autoware_msgs.msg import TrafficLightResult, TrafficLightResultArray

from helpers.lanelet2 import load_lanelet2_map, get_stoplines_api_id

MQTT_TO_AUTOWARE_TFL_MAP = {
    "RED": 0,
    "RED/AMB": 0,
    "REDVAMB": 0,
    "RED\\AMB": 0,
    "AMBER-RED": 0,
    "AMBERRED" : 0,
    "YELLOW": 0,
    "AMBER": 0,
    "FLASH" : 2,
    "AMB FLASH" : 2,
    "AMBER FLASH" : 2,
    "GREEN FLASH": 1,
    "GREEN": 1,
    "OFF": 2,
    "UNKNOWN": 2
}

class MqttTrafficLightDetector:
    def __init__(self):

        # Node parameters
        self.mqtt_host = rospy.get_param('~mqtt_host')
        self.mqtt_port = rospy.get_param('~mqtt_port')
        self.mqtt_topic = rospy.get_param('~mqtt_topic')
        self.timeout = rospy.get_param('~timeout')
        self.rate = rospy.Rate(10) # 10hz
        self.id_string = rospy.get_param('~id_string')

        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")
        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")

        lanelet2_map = load_lanelet2_map(lanelet2_map_name, coordinate_transformer, use_custom_origin, utm_origin_lat, utm_origin_lon)
        self.stop_line_ids = get_stoplines_api_id(lanelet2_map)

        # MQTT traffic light status
        self.mqtt_status = {}

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', TrafficLightResultArray, queue_size=1, tcp_nodelay=True)

        client = paho.Client()
        client.on_message = self.on_message
        client.on_disconnect = self.on_disconnect
        client.on_connect = self.on_connect

        client.tls_set("/etc/ssl/certs/ca-certificates.crt")
        client.connect(self.mqtt_host, self.mqtt_port, keepalive=10)
        client.loop_start()


    def on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            rospy.logerr('%s - failed to connect to MQTT server %s:%d, return code: %d', rospy.get_name(), self.mqtt_host, self.mqtt_port, rc)
            return

        rospy.loginfo('%s - connected to MQTT server %s:%d', rospy.get_name(), self.mqtt_host, self.mqtt_port)
        client.subscribe(self.mqtt_topic)

    def on_disconnect(self, client, userdata, rc):
        rospy.logerr('%s - disconnected from MQTT server %s:%d, return code: %d', rospy.get_name(), self.mqtt_host, self.mqtt_port, rc)

    def on_message(self, client, userdata, msg):
        rospy.logdebug('%s - MQTT message recieved: %s, %s', rospy.get_name(), msg.topic, str(msg.payload))
        # collect all messages
        api_id = msg.topic
        self.mqtt_status[api_id] = json.loads(msg.payload)

    def combine_tfl_results_and_publish(self):
        """
        Combine extracted stop lines with api_id from the map with the traffic light status from mqtt
        """
        
        # combine self.stop_line_ids[line.id] and self.tfl_status[tl_id]
        try:

            # iterate over stoplines and create TrafficLightResultArray
            tfl_status = TrafficLightResultArray()
            tfl_status.header.stamp = rospy.Time.now()

            for lane_id, api_id in self.stop_line_ids.items():

                result_str = "UNKNOWN"
                result = MQTT_TO_AUTOWARE_TFL_MAP[result_str]

                # extract status from mqtt_status if key exits
                if api_id in self.mqtt_status:
                    if self.mqtt_status[api_id]["timestamp"] < int((time.time() - self.timeout) * 1000):
                        rospy.logwarn('%s - timeout of stopline: %s, by %f seconds', rospy.get_name(), api_id, (self.mqtt_status[api_id]["timestamp"] - time.time() * 1000) / 1000)
                    else:
                        result_str = self.mqtt_status[api_id]["status"]
                        if result_str in MQTT_TO_AUTOWARE_TFL_MAP:
                            result = MQTT_TO_AUTOWARE_TFL_MAP[result_str]
                        else:
                            rospy.logwarn('%s - unknown stopline state: %s', rospy.get_name(), result_str)

                tfl_result = TrafficLightResult()
                tfl_result.light_id = 0
                tfl_result.lane_id = lane_id
                tfl_result.recognition_result = result
                tfl_result.recognition_result_str = result_str + self.id_string
                tfl_status.results.append(tfl_result)

            self.tfl_status_pub.publish(tfl_status)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def run(self):
        while not rospy.is_shutdown():
            self.combine_tfl_results_and_publish()
            try:
                self.rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass



if __name__ == '__main__':
    rospy.init_node('mqtt_traffic_light_detector', log_level=rospy.INFO)
    node = MqttTrafficLightDetector()
    node.run()