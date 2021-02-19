#include "manipulator_skills/skills/service_servers/update_param.hpp"

namespace manipulator_skills
{

UpdateParam::UpdateParam(
    const ros::NodeHandle &private_node_handle,
    std::string service_name):
    ManipulatorSkill(service_name),
    pnh_(private_node_handle),
    service_name_(service_name)
  {
    this->initialize();
  }

UpdateParam::~UpdateParam()
{
}

void UpdateParam::initialize()
{
    service_ = pnh_.advertiseService(service_name_, &UpdateParam::executeCB, this);
    ROS_INFO_STREAM_NAMED(getName(),getName() << ": waitng for call" );
}

bool UpdateParam::executeCB(man_msgs::UpdateParam::Request  &req,
                        man_msgs::UpdateParam::Response &res)
{
    std::string topic_name = req.topic;
    std::string data_type = req.data_type;

    std::vector<float_t> value_double;
    std::vector<int8_t> value_int;
    std::vector<std::string> value_string;
    std::vector<std::string> all_params;
    std::vector<std::string> labels;

    pnh_.getParamNames(all_params);

    for (unsigned i=0; i<all_params.size(); i++)
    {
        // get requited topic
        std::size_t found = all_params.at(i).find(topic_name);
        if (found!=std::string::npos)
        {
            if(data_type.compare("double") == 0)
            {            
                double val;
                std::string label;

                pnh_.getParam(all_params.at(i),val);
                value_double.push_back(val);

                label = all_params.at(i).substr(found + topic_name.size()+1);
                labels.push_back(label); 
                ROS_INFO_STREAM_NAMED(getName(), getName() << ": label: " << label << " value: " << val);
            }

            if(data_type.compare("bool") == 0)
            {            
                bool val;
                std::string label;

                pnh_.getParam(all_params.at(i),val);
                value_int.push_back(val);

                label = all_params.at(i).substr(found + topic_name.size()+1);
                labels.push_back(label); 
                ROS_INFO_STREAM_NAMED(getName(), getName() << ": label: " << label << " value: " << val);
            }
            
            if(data_type.compare("int") == 0)
            {            
                int val;
                std::string label;

                pnh_.getParam(all_params.at(i),val);
                value_int.push_back(val);

                label = all_params.at(i).substr(found + topic_name.size()+1);
                labels.push_back(label); 
                ROS_INFO_STREAM_NAMED(getName(), getName() << ": label: " << label << " value: " << val);
            }

            if(data_type.compare("string") == 0)
            {            
                std::string val;
                std::string label;

                pnh_.getParam(all_params.at(i),val);
                value_string.push_back(val);

                label = all_params.at(i).substr(found + topic_name.size()+1);
                labels.push_back(label); 
                ROS_INFO_STREAM_NAMED(getName(), getName() << ": label: " << label << " value: " << val);
            }
        }
    }
    
    res.label = labels;
    res.value_double = value_double;
    res.value_string = value_string;
    res.value_int = value_int;
    return true;
}
} //namespace