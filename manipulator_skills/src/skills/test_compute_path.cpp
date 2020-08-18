#include "manipulator_skills/skills/compute_path_skill.hpp"
#include <string>

// int main(int argc, char** argv) {
//   std::string node_name = "computePathServerTest";
//   ros::init(argc, argv, node_name);


//   ComputePathSkillAction computePathtest();
//   ROS_INFO("Initialized a multi-thread node.");
// //   ros::MultiThreadedSpinner s(4);   // Use 4 threads
//   ROS_INFO_STREAM("Main loop in thread:" << boost::this_thread::get_id());
//     ros::spin();

// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ComputePathSkillAction");
  std::string name = "ComputePathSkillAction";

  bool create_client = false;

  manipulator_skills::ArmComputePathSkill ArmComputePathSkill(name);

  actionlib::SimpleActionClient<man_msgs::ComputePathSkillAction> ac("test_client", true);
  ROS_INFO("Waiting for action server to start.");

  man_msgs::ComputePathSkillGoal goal;
  

  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (ros::ok() && create_client == false) 
  {
    ROS_INFO("started!");

    ac.sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout)
    {
      ROS_INFO("unknown");
    }
    else
      ROS_INFO("Action did not finish before the time out.");
  }

  return 0;
}

