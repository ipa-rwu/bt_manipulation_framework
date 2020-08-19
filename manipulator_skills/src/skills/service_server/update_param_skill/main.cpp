#include "manipulator_skills/skills/update_param.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "update_param");
    ros::NodeHandle nh("");

    ros::AsyncSpinner spinner(1); 
    spinner.start();

    manipulator_skills::UpdateParam UpdateParam(nh);

    while (ros::ok())
    {

    }
    return 0;
}