#include "man_behavior_tree/bt_engine.hpp"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include <memory>
#include <string>

namespace man_behavior_tree
{
//  from nav2
BT_Engine::BT_Engine(const std::vector<std::string> & plugin_libraries)
{
  BT::SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }
}

//  from nav2
BT::Tree
BT_Engine::buildTreeFromText(
  const std::string & xml_string,
  BT::Blackboard::Ptr blackboard)
{
  BT::XMLParser p(factory_);
  p.loadFromText(xml_string);
  return p.instantiateTree(blackboard);
}

} // namespace man_behavior_tree

