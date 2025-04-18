#!/usr/bin/env python3

import time
import rospy
import math
from std_msgs.msg import ColorRGBA,Float64MultiArray
from novatel_oem7_msgs.msg import INSPVA, BESTPOS
from jsk_rviz_plugins.msg import OverlayText

INSPVA_STATUS = {
    0: "INS_INACTIVE",
    1: "INS_ALIGNING",
    2: "INS_HIGH_VARIANCE",
    3: "INS_SOLUTION_GOOD",
    6: "INS_SOLUTION_FREE",
    7: "INS_ALIGNMENT_COMPLETE",
    8: "DETERMINING_ORIENTATION",
    9: "WAITING_INITIALPOS",
    10: "WAITING_AZIMUTH",
    11: "INITIALIZING_BIASES",
    12: "MOTION_DETECT",
    14: "WAITING_ALIGNMENTORIENTATION"
    }

BESTPOS_POS_TYPE = {
    0: "NONE",
    1: "FIXEDPOS",
    2: "FIXEDHEIGHT",
    8: "DOPPLER_VELOCITY",
    16: "SINGLE",
    17: "PSRDIFF",
    18: "WAAS",
    19: "PROPAGATED",
    32: "L1_FLOAT",
    34: "NARROW_FLOAT",
    48: "L1_INT",
    49: "WIDE_INT",
    50: "NARROW_INT",
    51: "RTK_DIRECT_INS",
    52: "INS_SBAS",
    53: "INS_PSRSP",
    54: "INS_PSRDIFF",
    55: "INS_RTKFLOAT",
    56: "INS_RTKFIXED",
    68: "PPP_CONVERGING",
    69: "PPP",
    70: "OPERATIONAL",
    71: "WARNING",
    72: "OUT_OF_BOUNDS",
    73: "INS_PPP_CONVERGING",
    74: "INS_PPP",
    77: "PPP_BASIC_CONVERGING",
    78: "PPP_BASIC",
    79: "INS_PPP_BASIC_CONVERGING",
    80: "INS_PPP_BASIC"
    }

INS_SOLUTION_GOOD = 3
INS_RTKFIXED = 56

class NovatelOem7Visualizer:
    def __init__(self):

        # Parameters
        self.number_of_satellites_good = rospy.get_param("~number_of_satellites_good")
        self.number_of_satellites_bad = rospy.get_param("~number_of_satellites_bad")
        self.location_accuracy_stdev_good = rospy.get_param("~location_accuracy_stdev_good")
        self.location_accuracy_stdev_bad = rospy.get_param("~location_accuracy_stdev_bad")
        self.differential_age_good = rospy.get_param("~differential_age_good")
        self.differential_age_bad = rospy.get_param("~differential_age_bad")
        self.exec_time_pub = rospy.Publisher(
                              f"/{rospy.get_name()}/exec_time_with_stamp",
                              Float64MultiArray,
                              queue_size=10
                             ) 
        # Publishers
        self.gnss_general_pub = rospy.Publisher('gnss_general', OverlayText, queue_size=1)
        self.gnss_detailed_pub = rospy.Publisher('gnss_detailed', OverlayText, queue_size=1)

        # Subscribers
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.inspva_callback, queue_size=1)
        rospy.Subscriber('/novatel/oem7/bestpos', BESTPOS, self.bestpos_callback, queue_size=1)

        # Internal parameters
        self.inspva_status_text = ""

    def inspva_callback(self, msg):

        if msg.status.status == INS_SOLUTION_GOOD:
            inspva_status_text = "<span style='color: white;'>{}</span>\n".format(INSPVA_STATUS[msg.status.status])
        else:
            if msg.status.status not in INSPVA_STATUS:
                inspva_status_text = "<span style='color: red;'>{}</span>\n".format("Unknown: " + str(msg.status.status))
            else:
                inspva_status_text = "<span style='color: yellow;'>{}</span>\n".format(INSPVA_STATUS[msg.status.status])

        self.inspva_status_text = inspva_status_text


    def bestpos_callback(self, msg):

        ################# inspva_status
        start_time = time.time()
        try:
         stampe = msg.header.stamp.to_sec()
         if stampe == 0.0:
          raise ValueError("Zero stamp")
        except:
         stampe = rospy.get_rostime().to_sec()
        inspva_status_text = "INS Status: "
        if self.inspva_status_text == "":
            inspva_status_text += "<span style='color: red;'>{}</span>\n".format("No INS status received")
        else:
            inspva_status_text += self.inspva_status_text
        
        ################# bestpos_pos_type

        bestpos_pos_type_text = "Position type: "
        if msg.pos_type.type == INS_RTKFIXED:
            bestpos_pos_type_text += "<span style='color: white;'>{}</span>\n".format(BESTPOS_POS_TYPE[msg.pos_type.type])
        else:
            if msg.pos_type.type not in BESTPOS_POS_TYPE:
                bestpos_pos_type_text += "<span style='color: red;'>{}</span>\n".format("Unknown: " + str(msg.pos_type.type))
            else:
                bestpos_pos_type_text += "<span style='color: yellow;'>{}</span>\n".format(BESTPOS_POS_TYPE[msg.pos_type.type])

        ################# num_sol_svs
        num_sol_svs_text = "Num. satellites: "
        if msg.num_sol_svs >= self.number_of_satellites_good:
            num_sol_svs_text += "<span style='color: white;'>{}</span>\n".format(msg.num_sol_svs)
        elif msg.num_sol_svs < self.number_of_satellites_bad:
            num_sol_svs_text += "<span style='color: red;'>{}</span>\n".format(msg.num_sol_svs)
        else:
            num_sol_svs_text += "<span style='color: yellow;'>{}</span>\n".format(msg.num_sol_svs)

        ################# loc_stdev
        location_stdev_text = "Location stdev: "
        location_stdev = math.sqrt(msg.lat_stdev**2 + msg.lon_stdev**2)

        if location_stdev <= self.location_accuracy_stdev_good:
            location_stdev_text += "<span style='color: white;'>{:.2f} m</span>\n".format(location_stdev)
        elif location_stdev > self.location_accuracy_stdev_bad:
            location_stdev_text += "<span style='color: red;'>{:.2f} m</span>\n".format(location_stdev)
        else:
            location_stdev_text += "<span style='color: yellow;'>{:.2f} m</span>\n".format(location_stdev)

        ################# diff_age
        diff_age_text = "Differential age: "

        if msg.diff_age <= self.differential_age_good:
            diff_age_text += "<span style='color: white;'>{:.2f} s</span>\n".format(msg.diff_age)
        elif msg.diff_age > self.differential_age_bad:
            diff_age_text += "<span style='color: red;'>{:.2f} s</span>\n".format(msg.diff_age)
        else:
            diff_age_text += "<span style='color: yellow;'>{:.2f} s</span>\n".format(msg.diff_age)


        ################# gnss_general status
        if msg.pos_type.type == INS_RTKFIXED and location_stdev < self.location_accuracy_stdev_good and msg.diff_age < self.differential_age_bad and msg.num_sol_svs > self.number_of_satellites_bad:
            gnss_general_text = "<span style='color: white;'>OK</span>"
        else:
            gnss_general_text = "<span style='color: yellow;'>Warning</span>"
        
        exec_duration = time.time() - start_time
        timing_msg = Float64MultiArray()
        timing_msg.data = [stampe, exec_duration]
        self.exec_time_pub.publish(timing_msg)

        rospy.loginfo(f"[{rospy.get_name()}] Exec time: {exec_duration:.6f}s | Stamp: {stampe:.3f}")
        self.publish_gnss_general(gnss_general_text)
        self.publish_gnss_detailed(inspva_status_text + bestpos_pos_type_text + num_sol_svs_text + location_stdev_text + diff_age_text)

       
    def publish_gnss_general(self, gnss_general_text):

        gnss_general = OverlayText()
        gnss_general.text = "GNSS: " + gnss_general_text

        self.gnss_general_pub.publish(gnss_general)


    def publish_gnss_detailed(self, gnss_detailed_text):

        # gnss_detailed
        gnss_detailed = OverlayText()
        gnss_detailed.text = "<span style='font-style: bold; color: white;'>GNSS:</span>\n" + gnss_detailed_text

        self.gnss_detailed_pub.publish(gnss_detailed)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('novatel_oem7_visualizer', log_level=rospy.INFO)
    node = NovatelOem7Visualizer()
    node.run()
