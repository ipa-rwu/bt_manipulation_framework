#ifndef MAN_BEHAVIOR_TREE_NODES_WEBOTS_ELEMENTS_
#define MAN_BEHAVIOR_TREE_NODES_WEBOTS_ELEMENTS_

#include <webots_ros/BoolStamped.h>
#include <dirent.h>

#include <fstream>



namespace man_behavior_tree_nodes
{

class WebotsSkills 
{
public:

std::string fixName(); 


int getProcIdByName(std::string procName);

std::string fixedNameString(const std::string &name);

private:
    std::string cmdPath_;
};

}   //namespace


#endif