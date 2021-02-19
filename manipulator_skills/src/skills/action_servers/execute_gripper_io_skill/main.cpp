#include "ros/ros.h"
#include "manipulator_skills/skills/action_servers/execute_gripper_io.hpp"
#include <manipulator_skills/skill_names.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "execute_gripper_io");
    std::string group_name, service_name;
    std::string ns = ros::this_node::getName();
    ros::param::param<std::string>(ns + "/group_name", group_name, "manipulator");
    ros::param::param<std::string>(ns + "/service_name", service_name, manipulator_skills::EXECUTE_GRIPPER_IO_NAME);
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    manipulator_skills::ExecuteGripperIOSkill ExecuteGripperIOSkill(group_name, service_name);

    while (ros::ok())
    {

    }
    return 0;
}