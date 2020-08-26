#include "manipulator_skills/skills/find_objects.hpp"
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "FindObjectsSkillAction");
    ros::NodeHandle nh("");

    manipulator_skills::FindObjectsSkill FindObjectsSkill(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok())
    {
      
    }
    return 0;
}

