#include "manipulator_skills/skills/update_arm_state.hpp"
#include "manipulator_skills/skill_names.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "update_arm_state");
    ros::NodeHandle nh("");
    std::string group_name = "manipulator";

    ros::AsyncSpinner spinner(1); 
    spinner.start();

    manipulator_skills::ArmUpdateState ArmUpdateStateSkill(group_name, nh);

    while (ros::ok())
    {

    }
    return 0;
}