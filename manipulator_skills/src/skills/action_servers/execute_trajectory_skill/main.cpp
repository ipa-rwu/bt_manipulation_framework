#include "manipulator_skills/skills/execute_trajectory_skill.hpp"
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ExecuteTrajectorySkillAction");
    std::string group_name = "manipulator";

    manipulator_skills::ArmExecuteTrajectorySkill ArmExecuteTrajectorySkill(group_name);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok())
    {
      
    }
    return 0;
}

