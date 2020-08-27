#ifndef MAN_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
#define MAN_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/time.h"
namespace BT
{

// The follow templates are required when using these types as parameters
// in BT XML as input


template<>
inline geometry_msgs::Point convertFromString(const StringView key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3) {
    throw std::runtime_error("invalid number of fields for point attribute)");
  } else {
    geometry_msgs::Point position;
    position.x = BT::convertFromString<double>(parts[0]);
    position.y = BT::convertFromString<double>(parts[1]);
    position.z = BT::convertFromString<double>(parts[2]);
    return position;
  }
}

template<>
inline geometry_msgs::Quaternion convertFromString(const StringView key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 4) {
    throw std::runtime_error("invalid number of fields for orientation attribute)");
  } else {
    geometry_msgs::Quaternion orientation;
    orientation.x = BT::convertFromString<double>(parts[0]);
    orientation.y = BT::convertFromString<double>(parts[1]);
    orientation.z = BT::convertFromString<double>(parts[2]);
    orientation.w = BT::convertFromString<double>(parts[3]);
    return orientation;
  }
}


template <> 
inline geometry_msgs::PoseStamped convertFromString(const StringView key)
{
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 10) 
  {
    throw std::runtime_error("invalid number of fields for orientation attribute)");
  } 
  else
  {
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.seq = convertFromString<int>(parts[0]);
    poseStamped.header.stamp = ros::Time(convertFromString<int64_t>(parts[1]));
    poseStamped.header.frame_id = convertFromString<std::string>(parts[2]);
    poseStamped.pose.position.x = convertFromString<double>(parts[3]);
    poseStamped.pose.position.y = convertFromString<double>(parts[4]);
    poseStamped.pose.position.z = convertFromString<double>(parts[5]);
    poseStamped.pose.orientation.x = convertFromString<double>(parts[6]);
    poseStamped.pose.orientation.y = convertFromString<double>(parts[7]);
    poseStamped.pose.orientation.z = convertFromString<double>(parts[8]);
    poseStamped.pose.orientation.w = convertFromString<double>(parts[9]);
    return poseStamped;
  }    
}

template<>
inline std::chrono::milliseconds convertFromString<std::chrono::milliseconds>(const StringView key)
{
  return std::chrono::milliseconds(std::stoul(key.data()));
}

}  // namespace BT

#endif  // NAV2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
