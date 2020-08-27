
#include "man_bt_operator/bt_manipulator.hpp"

namespace man_bt_operator
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
    ROS_ERROR("Please provide bt_xml_file");
    ros::shutdown();
  }

    // wrap bt_engine
    bt_ = std::make_unique<man_behavior_tree_nodes::BT_Engine>(plugin_lib_names_);

    // add items to blackboard
    // Create the blackboard that will be shared by all of the nodes in the tree
    blackboard_ = BT::Blackboard::create();

    // Put items on the blackboard
    blackboard_->set<std::map<std::string, _Float32>>("param_float", param_float_);  // NOLINT
    blackboard_->set<ros::NodeHandle>("node_handle", pnh_);  // NOLINT

    
    // load bt xml file
    if(!loadBehaviorTree(default_bt_xml_filename_))
    {
      ROS_ERROR("Error loading XML file: %s", default_bt_xml_filename_.c_str());
      ros::shutdown();

    }

    #ifdef ZMQ_FOUND
      BT::PublisherZMQ publisher_zmq(tree_);
    #endif

    // This logger prints state changes on console
    BT::StdCoutLogger logger_cout(tree_);
    printTreeRecursively(tree_.rootNode());  

    bt_->run(&tree_);
    
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
    ROS_ERROR( "Couldn't open input XML file: %s", bt_xml_filename.c_str());
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



} //namespace