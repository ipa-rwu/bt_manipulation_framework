#ifndef MANIPULATOR_SKILLS_UPDATE_PARAM_SKILL_
#define MANIPULATOR_SKILLS_UPDATE_PARAM_SKILL_

#include "ros/ros.h"
#include "manipulator_skills/skill.hpp"
#include "man_msgs/UpdateParam.h"
#include "man_msgs/ParameterMsg.h"

namespace manipulator_skills
{
class UpdateParam  : public ManipulatorSkill
{
public:
  UpdateParam(
    const ros::NodeHandle &private_node_handle,
    std::string service_name);
  ~UpdateParam();

  void initialize() override;

private:
    bool executeCB(man_msgs::UpdateParam::Request  &req,
                    man_msgs::UpdateParam::Response &res);

    ros::ServiceServer service_;

    std::string service_name_;
  
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
};
} //namespace manipulator_skills

#endif