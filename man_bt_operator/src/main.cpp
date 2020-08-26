#include "man_bt_operator/bt_manipulator.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bt_manipulator");
    ros::NodeHandle nh("");
    
    std::string namespace_param = "/bt_param";

    ros::AsyncSpinner spinner(1); 
    spinner.start();

    man_bt_operator::BT_Manipulator BT_Manipulator(nh, namespace_param);

    return 0;
}