#include <string>
#include "man_bt_operator/bt_ros_bridge.hpp"


namespace bt_ros_bridge 
{
    
BTROSBridge::BTROSBridge(const ros::NodeHandle &node_handle,
                                 const ros::NodeHandle &private_node_handle)
    :_nh(node_handle), _pnh(private_node_handle)
    {
        this->init();
    }
}