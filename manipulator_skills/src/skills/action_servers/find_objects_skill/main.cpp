#include "manipulator_skills/skills/action_servers/find_objects.hpp"
#include <string>
#include <manipulator_skills/skill_names.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "FindObjectsSkillAction");
    std::string service_name;
    std::string ns = ros::this_node::getName();
    ros::param::param<std::string>(ns + "/service_name", service_name, manipulator_skills::FIND_OBJECTS_NAME);
    ros::NodeHandle nh("");
    manipulator_skills::FindObjectsSkill FindObjectsSkill(nh, service_name);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
      
    }
    return 0;
}