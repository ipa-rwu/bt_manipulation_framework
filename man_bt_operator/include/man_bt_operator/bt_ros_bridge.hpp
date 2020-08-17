#ifndef BT_ROS_BRIDGE_HPP
#define BT_ROS_BRIDGE_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

namespace bt_ros_bridge
{
class BTROSBridge
{
public:
    
    BTROSBridge(const ros::NodeHandle &node_handle,
            const ros::NodeHandle &private_node_handle);
    
    ~BTROSBridge() = default;

    void init();

private:
    // public ros node handle
    ros::NodeHandle _nh;
    // private ros node handle
    ros::NodeHandle _pnh;
    std::string node_name_{"BT ROS bridge node"};

}; // class
} //namespace
#endif