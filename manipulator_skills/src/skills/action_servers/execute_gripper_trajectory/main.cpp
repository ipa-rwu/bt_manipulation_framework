#include "manipulator_skills/skills/action_servers/execute_gripper_trajectory.hpp"
#include <string>
#include <manipulator_skills/skill_names.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ExecuteGripperTrajectoryAction");
    std::string group_name, service_name;
    std::string ns = ros::this_node::getName();
    ros::param::param<std::string>(ns + "/group_name", group_name, "gripper");
    ros::param::param<std::string>(ns + "/service_name", service_name, manipulator_skills::EXECUTE_GRIPPER_TRAJECTORY_NAME);
    manipulator_skills::GripperExecuteTrajectorySkill GripperExecuteTrajectorySkill(group_name, service_name);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok())
    {
      
    }
    return 0;
}