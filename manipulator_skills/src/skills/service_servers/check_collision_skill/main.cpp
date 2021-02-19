#include "manipulator_skills/skills/check_collision.hpp"
#include "ros/ros.h"
#include <manipulator_skills/skill_names.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "check_collision");
    ros::NodeHandle nh("");
    std::string service_name;
    std::string ns = ros::this_node::getName();
    ros::param::param<std::string>(ns + "/service_name", service_name, manipulator_skills::CHECK_COLLISION);
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    manipulator_skills::CheckCollision CheckCollision(nh);

    while (ros::ok())
    {

    }
    return 0;
}