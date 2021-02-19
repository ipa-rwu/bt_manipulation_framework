#ifndef MANIPULATOR_SKILLS_SET_FLAG_SKILL_
#define MANIPULATOR_SKILLS_SET_FLAG_SKILL_

#include "ros/ros.h"
#include "manipulator_skills/skill.hpp"

#include "man_msgs/SetParameter.h"
#include "man_msgs/ParameterMsg.h"

namespace manipulator_skills
{

class SetFlagForTask  : public ManipulatorSkill
{
public:
  SetFlagForTask(
    const ros::NodeHandle &private_node_handle);
  ~SetFlagForTask();

  void initialize() override;

private:
    bool executeCB(man_msgs::SetParameter::Request  &req,
                        man_msgs::SetParameter::Response &res);


    void getParam(std::vector<std::string> all_params,
                            std::string param_topic, 
                            std::vector<std::string> labels,
                            std::vector<int8_t> value_int);
    
    void setParam(std::string param_topic,
                std::string task_name,
                bool task_value);


    ros::ServiceServer service_;

    std::string service_name_;
  
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    // shared_ptr    

    std::string param_topic_{"task_parameter_server"};



};
} //namespace manipulator_skills




#endif