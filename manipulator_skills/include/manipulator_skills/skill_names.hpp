#ifndef MANIPULATOR_SKILLS_SKILL_NAMES_
#define MANIPULATOR_SKILLS_SKILL_NAMES_

#include <string>

namespace manipulator_skills
{
    static const std::string COMPUTE_PATH_NAME = "compute_path";  // name of 'compute path skill' action
    static const std::string UPDATE_ARM_STATE = "update_arm_state";  // name of 'update_arm_state' service
    static const std::string UPDATE_ARM_GOAL = "update_arm_goal";  // name of 'update_arm_goal' service
    static const std::string UPDATE_PARAM = "skill_update_param";  // name of 'update_param' service
    static const std::string FIND_OBJECTS_NAME = "find_objects";  // name of 'find_objects' action
    static const std::string EXECUTE_TRAJECTORY_NAME = "execute_trajectory_arm";  // name of 'find_objects' action
    static const std::string EXECUTE_GRIPPER_TRAJECTORY_NAME = "execute_gripper_trajectory";  // name of 'find_objects' action
    static const std::string SET_FLAG_FOR_TASK = "set_flag_for_task";
    static const std::string CHECK_COLLISION = "check_collision";
    static const std::string SET_PARAMETER = "skill_set_parameter_server";
    static const std::string EXECUTE_GRIPPER_IO_NAME = "skill_execute_gripper_io_server";
}
#endif