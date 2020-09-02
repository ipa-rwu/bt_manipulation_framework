#include "manipulator_skills/skills/set_flag_for_task.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_flag_for_task");
    ros::NodeHandle nh("");

    ros::AsyncSpinner spinner(1); 
    spinner.start();

    manipulator_skills::SetFlagForTask SetFlagForTask(nh);

    while (ros::ok())
    {

    }
    return 0;
}