#include "manipulator_skills/skills/update_arm_goal.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "update_arm_goal");
    ros::NodeHandle nh("");
    std::string group_name = "manipulator";

    ros::AsyncSpinner spinner(1); 
    spinner.start();

    manipulator_skills::ArmUpdateGoal ArmUpdateStateSkill(group_name, nh);

    while (ros::ok())
    {

    }
    return 0;
}