#ifndef MANIPULATOR_SKILLS_WEBOTS_ELEMENTS_
#define MANIPULATOR_SKILLS_WEBOTS_ELEMENTS_

#include <webots_ros/BoolStamped.h>
#include <dirent.h>

#include <fstream>



namespace manipulator_skills
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