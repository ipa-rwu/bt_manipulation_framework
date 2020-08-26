#include "manipulator_skills/skills/update_param.hpp"
#include "manipulator_skills/skill_names.hpp"

namespace manipulator_skills
{

UpdateParam::UpdateParam(
    const ros::NodeHandle &private_node_handle) :
    ManipulatorSkill(UPDATE_PARAM),
    pnh_(private_node_handle),
    service_name_(UPDATE_PARAM)
  {
    this->initialize();
  }

UpdateParam::~UpdateParam()
{

}

void UpdateParam::initialize()
{

    service_ = pnh_.advertiseService(UPDATE_PARAM, &UpdateParam::executeCB, this);
    ROS_INFO_STREAM_NAMED(getName(), "start service" );
}

bool UpdateParam::executeCB(man_msgs::UpdateParam::Request  &req,
                        man_msgs::UpdateParam::Response &res)
{
    ROS_INFO("get request");
    std::string topic_name = req.topic;
    std::string data_type = req.data_type;

    std::cout << "topic: " << topic_name; 
    std::cout << " data_type: " << data_type<<std::endl;
    std::vector<float_t> value_double;
    std::vector<int8_t> value_int;
    std::vector<std::string> value_string;
    std::vector<std::string> all_params;
    std::vector<std::string> labels;

    pnh_.getParamNames(all_params);
    // for (unsigned i=0; i<all_params.size(); i++)
    // {
    //     std::cout << ' ' << all_params.at(i);
    //     std::cout << '\n';
    // }

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
                std::cout <<" "<< label;
                std::cout << '\n';
            }

            if(data_type.compare("bool") == 0)
            {            
                bool val;
                std::string label;

                pnh_.getParam(all_params.at(i),val);
                value_int.push_back(val);

                label = all_params.at(i).substr(found + topic_name.size()+1);
                labels.push_back(label); 
            }
            
            if(data_type.compare("int") == 0)
            {            
                int val;
                std::string label;

                pnh_.getParam(all_params.at(i),val);
                value_int.push_back(val);

                label = all_params.at(i).substr(found + topic_name.size()+1);
                labels.push_back(label); 
            }

            if(data_type.compare("string") == 0)
            {            
                std::string val;
                std::string label;

                pnh_.getParam(all_params.at(i),val);
                value_string.push_back(val);

                label = all_params.at(i).substr(found + topic_name.size()+1);
                labels.push_back(label); 
                }
        }
    }
    
    res.label = labels;
    res.value_double = value_double;
    res.value_string = value_string;
    res.value_int = value_int;
    return true;
}

void UpdateParam::getParam(std::string topic_name, 
                            std::string data_type, 
                            std::vector<std::string> all_params, 
                            std::vector<std::string> labels,
                            std::vector<float_t> value_d,
                            std::vector<int8_t> value_i,
                            std::vector<std::string> value_s)
{
    pnh_.getParamNames(all_params);
    // get all parameter topic
    
    // for (unsigned i=0; i<all_params.size(); i++)
    // {
    //     // get requited topic
    //     std::size_t found = all_params.at(i).find(topic_name);
    //     if (found!=std::string::npos)
    //     {
    //         if(data_type.compare("double") == 0)
    //         {            
    //             double val;
    //             std::string label;

    //             pnh_.getParam(all_params.at(i),val);
    //             value_d.push_back(val);

    //             label = all_params.at(i).substr(found + topic_name.size()+1);
    //             labels.push_back(label); 

    //             std::cout <<" " << label;
    //             std::cout <<" "<< val;
    //             std::cout <<" "<< i;
    //             std::cout << '\n';
    //         }

            
    //         if(data_type.compare("bool") == 0)
    //         {            
    //             bool val;
    //             pnh_.getParam(all_params.at(i),val);
    //             std::cout <<" " << all_params.at(i).substr(found + topic_name.size()+1);
    //             std::cout <<" "<< val;
    //             std::cout << '\n';
    //             }

    //         if(data_type.compare("int") == 0)
    //         {            
    //             int val;
    //             pnh_.getParam(all_params.at(i),val);
    //             std::cout <<" " << all_params.at(i).substr(found + topic_name.size()+1);
    //             std::cout <<" "<< val;
    //             std::cout << '\n';
    //             }
    //         if(data_type.compare("string") == 0)
    //         {            
    //             std::string val;
    //             pnh_.getParam(all_params.at(i),val);
    //             std::cout <<" " << all_params.at(i).substr(found + topic_name.size()+1);
    //             std::cout <<" "<< val;
    //             std::cout << '\n';
    //             }
    //     }
    //     std::cout <<" "<< i;
    // }
    // std::cout << "finished";

}

} //namespace
