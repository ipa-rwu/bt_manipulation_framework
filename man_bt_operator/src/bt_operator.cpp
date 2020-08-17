#include "man_bt_operator/bt_operator.hpp"

namespace man_bt_operator
{

bool BT_Operator::loadBehaviorTree(const std::string & bt_xml_filename)
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