#include "man_bt_operator/bt_manipulator.hpp"

namespace bt_manipulator
{

BT_Manipulator::BT_Manipulator(
  const ros::NodeHandle &private_node_handle,
  std::string namespace_param):
  pnh_(private_node_handle),
  namespace_param_(namespace_param)
{
  this->initialize();
}

void BT_Manipulator::initialize()
{
  if(!pnh_.getParam(namespace_param_ + "/plugin_lib_names", plugin_lib_names_))
  {
    const std::vector<std::string> plugin_libs = {
    "man_update_param_service_client_node"
    };
    plugin_lib_names_ = plugin_libs;
  }

  if(!pnh_.getParam(namespace_param_ + "/bt_xml_file_name", default_bt_xml_filename_))
  {
    ROS_ERROR("Please provide bt_xml_file")
  }

    // wrap bt_engine
    bt_ = std::make_unique<man_behavior_tree::BT_Engine>(plugin_lib_names_);
    
    // load bt xml file
    if(!loadBehaviorTree(default_bt_xml_filename_))
    {
      ROS_ERROR("Error loading XML file: %s", default_bt_xml_filename_.c_str());
    }
}


BT_Manipulator::~BT_Manipulator()
{

}

bool BT_Manipulator::loadBehaviorTree(const std::string & bt_xml_filename)
{
  // Use previous BT if it is the existing one
  if (current_bt_xml_filename_ == bt_xml_filename) {
    return true;
  }

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(bt_xml_filename);

  if (!xml_file.good()) {
    // ROSINFO(get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
    return false;
  }

  auto xml_string = std::string(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>());

//   ROSINFO(get_logger(), "Behavior Tree file: '%s'", bt_xml_filename.c_str());
//   ROSINFO(get_logger(), "Behavior Tree XML: %s", xml_string.c_str());

  // Create the Behavior Tree from the XML input
  tree_ = bt_->buildTreeFromText(xml_string, blackboard_);
  current_bt_xml_filename_ = bt_xml_filename;

  return true;
}

}