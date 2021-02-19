#include "manipulator_skills/skills/service_servers/set_parameter.hpp"
#include <iostream>

namespace manipulator_skills
{

SetParameter::SetParameter(
    const ros::NodeHandle &private_node_handle,
    std::string service_name) :
    ManipulatorSkill(service_name),
    pnh_(private_node_handle),
    service_name_(service_name)
  {
    this->initialize();
  }

SetParameter::~SetParameter()
{
}

void SetParameter::initialize()
{
    service_ = pnh_.advertiseService(service_name_, &SetParameter::executeCB, this);
    ROS_INFO_STREAM_NAMED(getName(),getName() << ": waitng for call" );
}

bool SetParameter::executeCB(man_msgs::SetParameter::Request  &req,
                        man_msgs::SetParameter::Response &res)
{
    if(!req.param_topic.empty())
        param_topic_ = req.param_topic;
    else{
        ROS_WARN_STREAM_NAMED(getName(),getName() << ": only change param in blackboard" );
        res.result = true;
        return true;
    }

    if(!req.data_type.empty())
        data_type_ = req.data_type;
    else{
        ROS_ERROR_STREAM_NAMED(getName(),getName() << ": missing param data type" );
    }

    std::vector<std::string> all_params;
    std::vector<std::string> labels;
    
    getParam(param_topic_, data_type_);

    setParam(param_topic_, req.label, data_type_, req.param_float, req.param_bool);

    res.result = true;
    return true;
}

void SetParameter::getParam(std::string param_topic, 
                            std::string data_type)
{
    std::vector<std::string> all_params;
    pnh_.getParamNames(all_params);
    float val_float;
    bool val_bool;

    for (unsigned i=0; i<all_params.size(); i++)
    {
        // get required topic
        std::size_t found = all_params.at(i).find(param_topic);
        if (found!=std::string::npos)
        {
            std::string label;
            if(data_type.compare("double") == 0){
                // task value
                pnh_.getParam(all_params.at(i), val_float);
                //  lables = all task names
                label = all_params.at(i).substr(found + param_topic.size() + 1);
                ROS_DEBUG_STREAM_NAMED(getName(),getName() << ": label: " << label << " value: " << val_float);
            }
            if(data_type.compare("bool") == 0){
                // task value
                pnh_.getParam(all_params.at(i), val_bool);
                //  lables = all task names
                label = all_params.at(i).substr(found + param_topic.size() + 1);
                ROS_DEBUG_STREAM_NAMED(getName(),getName() << ": label: " << label << " value: " << val_bool);
            }
        }
    }
}
    
void SetParameter::setParam(std::string param_topic,
                std::string label,
                std::string data_type,
                float param_float,
                bool param_bool)
{
    if(data_type.compare("double") == 0){
        ROS_INFO_STREAM_NAMED(getName(), getName() << ": update "<< label<<" to "<< param_float);
        std::string command = "rosrun dynamic_reconfigure dynparam set_from_parameters";
        command = command + " " + param_topic + " _" + label + ":=" + std::to_string(param_float);
        system(command.c_str());
    }
    if(data_type.compare("bool") == 0){
        ROS_INFO_STREAM_NAMED(getName(), getName() << ": update "<< label<<" to "<< param_bool);
        std::string command = "rosrun dynamic_reconfigure dynparam set_from_parameters";
        command = command + " " + param_topic + " _" + label + ":=" + std::to_string(param_bool);
        system(command.c_str());
    }
}
} //namespace
