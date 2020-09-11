#include "manipulator_skills/skills/check_collision.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "check_collision");
    ros::NodeHandle nh("");

    ros::AsyncSpinner spinner(1); 
    spinner.start();

    manipulator_skills::CheckCollision CheckCollision(nh);

    while (ros::ok())
    {

    }
    return 0;
}