#ifndef MANIPULATOR_SKILLS_SKILL_
#define MANIPULATOR_SKILLS_SKILL_

#include <std_msgs/String.h>
#include <ros/ros.h>
namespace manipulator_skills
{
class ManipulatorSkill
{
public:
    ManipulatorSkill(const std::string& skill_name)
    : skill_name_(skill_name)
    {
    }
    
    virtual ~ManipulatorSkill()
    {   
    }

    virtual void initialize() = 0;

    const std::string& getName() const
    {
        return skill_name_;
    }
    
protected:
    std::string skill_name_;
    ros::NodeHandle root_node_handle_;
};
} //namespace

#endif //MANIPULATOR_SKILLS_SKILL_