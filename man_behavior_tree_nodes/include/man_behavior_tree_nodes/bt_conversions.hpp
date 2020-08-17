#ifndef MAN_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
#define MAN_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace BT
{

// The follow templates are required when using these types as parameters
// in BT XML as input


template<>
inline geometry_msgs::msg::Point convertFromString(const StringView key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3) {
    throw std::runtime_error("invalid number of fields for point attribute)");
  } else {
    geometry_msgs::msg::Point position;
    position.x = BT::convertFromString<double>(parts[0]);
    position.y = BT::convertFromString<double>(parts[1]);
    position.z = BT::convertFromString<double>(parts[2]);
    return position;
  }
}

template<>
inline geometry_msgs::msg::Quaternion convertFromString(const StringView key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 4) {
    throw std::runtime_error("invalid number of fields for orientation attribute)");
  } else {
    geometry_msgs::msg::Quaternion orientation;
    orientation.x = BT::convertFromString<double>(parts[0]);
    orientation.y = BT::convertFromString<double>(parts[1]);
    orientation.z = BT::convertFromString<double>(parts[2]);
    orientation.w = BT::convertFromString<double>(parts[3]);
    return orientation;
  }
}

template<>
inline std::chrono::milliseconds convertFromString<std::chrono::milliseconds>(const StringView key)
{
  return std::chrono::milliseconds(std::stoul(key.data()));
}

}  // namespace BT

#endif  // NAV2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
