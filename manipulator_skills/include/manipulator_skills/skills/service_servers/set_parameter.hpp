#ifndef MANIPULATOR_SKILLS_SET_FLAG_SKILL_
#define MANIPULATOR_SKILLS_SET_FLAG_SKILL_

#include "ros/ros.h"
#include "manipulator_skills/skill.hpp"

#include "man_msgs/SetParameter.h"

namespace manipulator_skills
{

class SetParameter: public ManipulatorSkill
{
public:
  SetParameter(
    const ros::NodeHandle &private_node_handle,
    std::string service_name);
  ~SetParameter();

  void initialize() override;

private:
    bool executeCB(man_msgs::SetParameter::Request  &req,
                        man_msgs::SetParameter::Response &res);

    void getParam(std::string param_topic, 
                  std::string data_type);
    
    void setParam(std::string param_topic,
                std::string label,
                std::string data_type,
                float param_value,
                bool param_bool);

    ros::ServiceServer service_;

    std::string service_name_;
  
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    // shared_ptr    

    std::string param_topic_;
    std::string data_type_;
};
} //namespace manipulator_skills

#endif