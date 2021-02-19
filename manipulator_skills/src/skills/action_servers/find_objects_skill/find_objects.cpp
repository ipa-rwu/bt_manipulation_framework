#include "manipulator_skills/skills/action_servers/find_objects.hpp"
#include "manipulator_skills/skill_names.hpp"

namespace manipulator_skills
{
FindObjectsSkill::FindObjectsSkill(const ros::NodeHandle &private_node_handle,
    std::string service_name) :
    ManipulatorSkill(service_name),
    pnh_(private_node_handle),
    service_name_(service_name)
  {
    this->initialize();
  }

FindObjectsSkill::~FindObjectsSkill()
{
}

void FindObjectsSkill::initialize()
{
    // start the move action server
    as_.reset(new FindObjectServer(root_node_handle_, service_name_, 
    boost::bind(&FindObjectsSkill::executeCB, this, _1), false));

    armarker_client_ = pnh_.serviceClient<ar_marker_detector::getMarkerPose>(armarker_srv_name_);

    as_->start();
    // ROS_INFO_STREAM_NAMED(getName(), "start action" );
}

void FindObjectsSkill::initialpose(geometry_msgs::Pose ps)
{
    ps.position.x = 0; 
    ps.position.y = 0; 
    ps.position.z = 0; 
    ps.orientation.x = 0; 
    ps.orientation.y = 0; 
    ps.orientation.z = 0; 
    ps.orientation.w = 0; 
}

void FindObjectsSkill::executeCB(const man_msgs::FindObjectsGoalConstPtr& goal)
{
    // ROS_INFO_NAMED(getName(), "[Find Objects] request received");

    initialpose(marker_posestamped_.pose);

    //  get marker pose
    if(getPose(goal, marker_posestamped_))
    {
      // ROS_INFO_STREAM_NAMED(getName(), "[goal] marker_id: "<< goal->marker_id);
      // ROS_INFO_STREAM_NAMED(getName(), "[goal] container_a.pose: "<< goal->container_a);
      // ROS_INFO_STREAM_NAMED(getName(), "[goal] container_b.pose: "<< goal->container_b);
      // ROS_INFO_STREAM_NAMED(getName(), "[response] marker_pose: "<< marker_posestamped_.pose);

      if(ComparePose(marker_posestamped_.pose, goal->container_a, goal->container_b))
      {
        action_res_.container_pose.pose = goal->container_a;
      }
      else
      {
        action_res_.container_pose.pose = goal->container_b;
      }
      ROS_INFO_STREAM_NAMED(getName(), getName() << ": container_pose: "<< action_res_.container_pose);
      ROS_INFO_STREAM_NAMED(getName(), getName() << ": marker: "<< action_res_.marker_pose);

      action_res_.marker_pose = marker_posestamped_;
      action_res_.marker_pose.header.frame_id = goal->frame_id;

      action_res_.marker_pose.header.frame_id = "world";

      action_res_.container_pose.header.frame_id = goal->frame_id;
      action_res_.container_pose.header.frame_id = "world";
      const std::string response = "SUCCESS";
      as_->setSucceeded(action_res_, response);
    }
    else
    {
      const std::string response = "FAILURE";
      as_->setAborted(action_res_, response);
    }
}

bool FindObjectsSkill::getPose(const man_msgs::FindObjectsGoalConstPtr& goal,
                                         geometry_msgs::PoseStamped &posestamp)
{
    armarker_srv_.request.ar_marker_id = goal->marker_id;
    if (armarker_client_.call(armarker_srv_))
    {
      posestamp.header.stamp = ros::Time::now();
      posestamp.pose = armarker_srv_.response.pose;   
      posestamp = armarker_srv_.response.poseStamped;   
      return true;
    }
    ROS_WARN_STREAM_NAMED(getName(), getName() <<": Didn't get marker pose");
    return false;
}

bool FindObjectsSkill::ComparePose(geometry_msgs::Pose A, geometry_msgs::Pose B, geometry_msgs::Pose C)
{
    double AB =
    ( A.position.x-B.position.x ) * ( A.position.x-B.position.x ) +
    ( A.position.y-B.position.y ) * ( A.position.y-B.position.y ) +
    ( A.position.z-B.position.z ) * ( A.position.z-B.position.z );
    // ROS_INFO_STREAM_NAMED(getName(), "[ComparePose] AB: "<< AB);
    double AC =
    ( A.position.x-C.position.x ) * ( A.position.x-C.position.x ) +
    ( A.position.y-C.position.y ) * ( A.position.y-C.position.y ) +
    ( A.position.z-C.position.z ) * ( A.position.z-C.position.z );
    // ROS_INFO_STREAM_NAMED(getName(), "[ComparePose] AC: "<< AC);

    if (AB > AC)
    {
      return true;
    }
    else
    {
      return false; 
    }

}
} // namespacev