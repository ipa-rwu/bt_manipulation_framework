#include "manipulator_skills/skills/compute_path_skill.hpp"
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ComputePathSkillAction");
    std::string group_name = "manipulator";

    manipulator_skills::ArmComputePathSkill ArmComputePathSkill(group_name);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok())
    {
      
    }
    return 0;
}

