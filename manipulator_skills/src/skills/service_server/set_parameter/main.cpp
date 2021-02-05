#include "manipulator_skills/skills/set_parameter.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_parameter");
    ros::NodeHandle nh("");

    ros::AsyncSpinner spinner(1); 
    spinner.start();

    manipulator_skills::SetParameter SetParameter(nh);

    while (ros::ok())
    {

    }
    return 0;
}