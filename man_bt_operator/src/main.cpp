#include "man_bt_operator/bt_manipulator.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bt_manipulator");
    ros::NodeHandle nh("");
    
    std::string bt_namespace_param = "/bt_param";
    std::string robot_namespace_param = "/robot_param";

    ros::AsyncSpinner spinner(1); 
    spinner.start();

    man_bt_operator::BT_Manipulator BT_Manipulator(nh, bt_namespace_param, robot_namespace_param);

    return 0;
}