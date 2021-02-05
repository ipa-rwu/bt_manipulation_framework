#ifndef MANIPULATOR_SKILLS_SET_FLAG_SKILL_
#define MANIPULATOR_SKILLS_SET_FLAG_SKILL_

#include "ros/ros.h"
#include "manipulator_skills/skill.hpp"

#include "man_msgs/SetFlagTask.h"
#include "man_msgs/ParameterMsg.h"

namespace manipulator_skills
{

class SetParameter: public ManipulatorSkill
{
public:
  SetParameter(
    const ros::NodeHandle &private_node_handle);
  ~SetParameter();

  void initialize() override;

private:
    bool executeCB(man_msgs::SetFlagTask::Request  &req,
                        man_msgs::SetFlagTask::Response &res);


    void getParam(std::vector<std::string> all_params,
                            std::string param_topic, 
                            std::vector<std::string> labels,
                            std::vector<float> value_float);
    
    void setParam(std::string param_topic,
                std::string task_name,
                float param_value);


    ros::ServiceServer service_;

    std::string service_name_;
  
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    // shared_ptr    

    std::string param_topic_{"arm_parameter_server"};



};
} //namespace manipulator_skills




#endif