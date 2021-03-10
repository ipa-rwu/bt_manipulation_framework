#!/usr/bin/env python
import rospy
import threading
from ar_track_alvar_msgs.msg import AlvarMarkers
from man_msgs.srv import GetMarkerPose, GetMarkerPoseResponse

Marker_poses = {}
Lock = {}
ArTopic = "ar_pose_marker"

def ar_marker_cb(msg, marker_id):
    flag = False
    with Lock[marker_id]:
        if len(msg.markers) == 0:
            Marker_poses[marker_id] = None 
        else:
            for i in msg.markers:
                rospy.logdebug("ar_marker_cb marker: " + str(i))
                if i.id == marker_id:
                    flag = True
                    Marker_poses[marker_id] = i.pose
                    Marker_poses[marker_id].header = i.header
    if flag == False:
        Marker_poses[marker_id] = None
        rospy.logdebug("ar_marker_cb Did not find " + str(marker_id))

def handle_call(req):
    res = GetMarkerPoseResponse()
    Lock[req.object_id] = threading.Lock()
    Lock[req.container_a_id] = threading.Lock()
    Lock[req.container_b_id] = threading.Lock()

    Marker_poses[req.object_id] = None
    Marker_poses[req.container_b_id] = None
    Marker_poses[req.container_a_id] = None

    for counter in range(0,3):
        if Marker_poses[req.container_a_id] == None and Marker_poses[req.container_b_id] == None:
            msg = rospy.wait_for_message(ArTopic, AlvarMarkers, timeout=None)
            ar_marker_cb(msg, req.container_a_id)
            ar_marker_cb(msg, req.container_b_id)
            rospy.sleep(0.5)
    if counter == 2 and Marker_poses[req.container_a_id] == None and Marker_poses[req.container_b_id] == None:
        raise rospy.ServiceException("Cannot find any containers")
    
    for counter in range(0,3):
        if Marker_poses[req.object_id] == None:
            msg = rospy.wait_for_message(ArTopic, AlvarMarkers, timeout=None)
            ar_marker_cb(msg, req.object_id)
    if counter == 2 and Marker_poses[req.object_id] == None:
        raise rospy.ServiceException("Cannot find object")

    res.object = Marker_poses[req.object_id]
    if Marker_poses[req.container_a_id] != None:
        res.container =  Marker_poses[req.container_a_id]
    if Marker_poses[req.container_b_id] != None:
        res.container =  Marker_poses[req.container_b_id]

    return res

def get_poses_server():
    rospy.init_node('find_objects_server')
    rate = rospy.Rate(100)
    rosparam = 'service_name'
    if rospy.has_param(rosparam):
        service_name = rospy.get_param(rosparam)
    else:
        service_name = 'skill_find_objects_server'
    rospy.Service(service_name, GetMarkerPose, handle_call)

    try:
        while not rospy.is_shutdown():        
            rate.sleep()

    except rospy.ROSException:
        pass    

if __name__ == "__main__":
    get_poses_server()
