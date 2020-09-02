#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from dynamic_parameter.cfg import PickPlaceArmConfig

def callback(config, level):
    #rospy.loginfo("""Reconfigure Request: {T2S1arm}, {T2S2arm}, {T2S3arm}, {T3S1arm}, {T3S2arm}, {T3S3arm}""".format(**config))
    return config
    
if __name__ == "__main__":
    rospy.init_node("arm_param", anonymous = False)

    srv = Server(PickPlaceArmConfig, callback)
    rospy.spin()