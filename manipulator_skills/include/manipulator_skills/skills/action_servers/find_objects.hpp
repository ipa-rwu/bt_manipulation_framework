#ifndef MANIPULATOR_SKILLS_FIND_OBJECTS_SKILL_
#define MANIPULATOR_SKILLS_FIND_OBJECTS_SKILL_

#include "ros/ros.h"
#include "manipulator_skills/skill.hpp"
#include <actionlib/server/simple_action_server.h>

#include "man_msgs/GetMarkerPose.h"
#include "man_msgs/FindObjectsAction.h"

namespace manipulator_skills
{

typedef actionlib::SimpleActionServer<man_msgs::FindObjectsAction> FindObjectServer;

class FindObjectsSkill  : public ManipulatorSkill
{
public:
  FindObjectsSkill(const ros::NodeHandle &private_node_handle,
                  std::string service_name);
  ~FindObjectsSkill();

  void initialize() override;

private:
  void executeCB(const man_msgs::FindObjectsGoalConstPtr &goal);

  void initialpose(geometry_msgs::Pose ps);

  bool getPose(const man_msgs::FindObjectsGoalConstPtr& goal,
                      geometry_msgs::PoseStamped& posestamp);

  bool ComparePose(geometry_msgs::Pose A, geometry_msgs::Pose B, geometry_msgs::Pose C);

  std::unique_ptr<FindObjectServer> as_;
  std::string service_name_;

  std::string armarker_srv_name_ = "/getMarkerPose";
  man_msgs::GetMarkerPose armarker_srv_;
  ros::ServiceClient armarker_client_;
  
  geometry_msgs::PoseStamped marker_posestamped_;
  man_msgs::FindObjectsResult action_res_;

  
  // public ros node handle
  ros::NodeHandle nh_;
  // private ros node handle
  ros::NodeHandle pnh_;

};
} //namespace manipulator_skills

#endif //MANIPULATOR_SKILLS_COMPUTER_PATH_SKILL_