#ifndef MANIPULATOR_SKILLS_UPDATE_PARAM_SKILL_
#define MANIPULATOR_SKILLS_UPDATE_PARAM_SKILL_

#include "ros/ros.h"
#include "manipulator_skills/skill.hpp"

#include "man_msgs/CheckCollisionService.h"
#include <webots_ros/BoolStamped.h>
#include <dirent.h>

#include <fstream>


namespace manipulator_skills
{

class CheckCollision: public ManipulatorSkill
{
public:
  CheckCollision(
    const ros::NodeHandle &private_node_handle);
  ~CheckCollision();

  void initialize() override;

private:
    bool executeCB(man_msgs::CheckCollisionService::Request  &req,
                        man_msgs::CheckCollisionService::Response &res);


    void getParam(std::string topic_name, 
                            std::string data_type, 
                            std::vector<std::string> all_params, 
                            std::vector<std::string> labels,
                            std::vector<float_t> value_d,
                            std::vector<int8_t> value_i,
                            std::vector<std::string> value_s);

    void TouchsensorCallback(const webots_ros::BoolStamped::ConstPtr& touchsensor_msg);

    // webots simulation
    void fixName();

    int getProcIdByName(std::string procName);

    std::string fixedNameString(const std::string &name);

    bool Touched();

    ros::ServiceServer service_;

    ros::Subscriber touch_sensor_sub_;

    std::string service_name_;

    std::string touch_sensor_topic_name_;

    std::string mRobotName_;

    bool result_touchsensor_;

    std::string cmdPath_;
    
  
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    // shared_ptr    
    


};
} //namespace manipulator_skills




#endif