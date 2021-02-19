#include "ros/ros.h"
#include "manipulator_skills/skills/service_servers/update_param.hpp"
#include "manipulator_skills/skill_names.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "update_param");
    ros::NodeHandle nh("");
    std::string service_name;
    std::string ns = ros::this_node::getName();
    ros::param::param<std::string>(ns + "/service_name", service_name, manipulator_skills::UPDATE_PARAM);
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    manipulator_skills::UpdateParam UpdateParam(nh, service_name);

    while (ros::ok())
    {

    }
    return 0;
}