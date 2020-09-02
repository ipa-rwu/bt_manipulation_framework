#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from dynamic_parameter.cfg import PickPlaceFlagConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {FHelp}, {FFoundObj}, {FPicked}, {FPlaced}""".format(**config))
    return config
    
if __name__ == "__main__":
    rospy.init_node("flag_config", anonymous = False)

    srv = Server(PickPlaceFlagConfig, callback)
    rospy.spin()