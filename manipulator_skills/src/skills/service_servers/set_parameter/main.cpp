#include "ros/ros.h"
#include "manipulator_skills/skills/service_servers/set_parameter.hpp"
#include <manipulator_skills/skill_names.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_parameter");
    ros::NodeHandle nh("");
    std::string service_name;
    std::string ns = ros::this_node::getName();
    ros::param::param<std::string>(ns + "/service_name", service_name, manipulator_skills::SET_PARAMETER);
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    manipulator_skills::SetParameter SetParameter(nh, service_name);

    while (ros::ok())
    {

    }
    return 0;
}