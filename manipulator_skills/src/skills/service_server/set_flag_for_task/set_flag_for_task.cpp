#include "manipulator_skills/skills/set_flag_for_task.hpp"
#include "manipulator_skills/skill_names.hpp"
#include <iostream>

namespace manipulator_skills
{

SetFlagForTask::SetFlagForTask(
    const ros::NodeHandle &private_node_handle) :
    ManipulatorSkill(SET_FLAG_FOR_TASK),
    pnh_(private_node_handle),
    service_name_(SET_FLAG_FOR_TASK)
  {
    this->initialize();
  }

SetFlagForTask::~SetFlagForTask()
{

}

void SetFlagForTask::initialize()
{

    service_ = pnh_.advertiseService(SET_FLAG_FOR_TASK, &SetFlagForTask::executeCB, this);
    ROS_INFO_NAMED(getName(), "start service: [%s] ", getName().c_str());
}

bool SetFlagForTask::executeCB(man_msgs::SetFlagTask::Request  &req,
                        man_msgs::SetFlagTask::Response &res)
{
    ROS_INFO_NAMED(getName(), "[%s] get request", getName().c_str());

    if(!req.param_topic.empty())
        param_topic_ = req.param_topic;

    std::vector<int8_t> value_int;
    std::vector<std::string> all_params;
    std::vector<std::string> labels;
    
    getParam(all_params, param_topic_, labels, value_int);

    setParam(param_topic_, req.task_name, req.success);

    res.result = true;
    return true;
}

void SetFlagForTask::getParam(std::vector<std::string> all_params,
                            std::string param_topic, 
                            std::vector<std::string> labels,
                            std::vector<int8_t> value_int)
{
    pnh_.getParamNames(all_params);
    std::cout << "getParam function: "  <<std::endl;

    for (unsigned i=0; i<all_params.size(); i++)
    {
        // get requited topic
        std::size_t found = all_params.at(i).find(param_topic);
        if (found!=std::string::npos)
        {
            bool val;
            std::string label;

            // task value
            pnh_.getParam(all_params.at(i), val);
            value_int.push_back(val);

            //  lables = all task names
            label = all_params.at(i).substr(found + param_topic.size() + 1);
            labels.push_back(label); 

            std::cout << "label: " << label << " value: " << val<<std::endl;                
        }
    }

}
    
void SetFlagForTask::setParam(std::string param_topic,
                std::string task_name,
                bool task_value)
{
    std::string value;
    if(task_value)
        value = "1";
    else
        value = "0";
    std::string command = "rosrun dynamic_reconfigure dynparam set_from_parameters";
    command = command + " " + param_topic + " _" + task_name + ":=" + value;
    std::cout << command << std::endl;
    system(command.c_str());
}

} //namespace
