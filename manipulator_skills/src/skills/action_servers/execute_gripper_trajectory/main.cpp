#include "manipulator_skills/skills/execute_gripper_trajectory.hpp"
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ExecuteGripperTrajectoryAction");
    std::string group_name = "gripper";

    manipulator_skills::GripperExecuteTrajectorySkill GripperExecuteTrajectorySkill(group_name);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok())
    {
      
    }
    return 0;
}

