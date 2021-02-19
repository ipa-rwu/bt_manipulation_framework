#include "manipulator_skills/skills/action_servers/compute_path_skill.hpp"
#include <string>
#include <manipulator_skills/skill_names.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ComputePathSkillAction");
    std::string group_name, service_name;
    std::string ns = ros::this_node::getName();
    ros::param::param<std::string>(ns + "/group_name", group_name, "manipulator");
    ros::param::param<std::string>(ns + "/service_name", service_name, manipulator_skills::COMPUTE_PATH_NAME);

    manipulator_skills::ArmComputePathSkill ArmComputePathSkill(group_name, service_name);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok())
    {
      
    }
    return 0;
}

