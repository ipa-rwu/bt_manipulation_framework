#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from dynamic_parameter.cfg import PickPlaceGripConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {T2S1gripper}, {T2S2gripper}, {T2S3gripper}, {T3S1gripper}, {T3S2gripper}, {T3S3gripper}""".format(**config))
    return config
    
if __name__ == "__main__":
    rospy.init_node("gripper_config", anonymous = False)

    srv = Server(PickPlaceGripConfig, callback)
    rospy.spin()