#ifndef MAN_BEHAVIOR_TREE_NODES_ARM_EXECUTE_TRAJECTORY_CLIENT_
#define MAN_BEHAVIOR_TREE_NODES_ARM_EXECUTE_TRAJECTORY_CLIENT_

#define ENABLE_SUBSCRIBE 1
#include <mutex> 
#include "man_behavior_tree_nodes/bt_action_client.hpp"
#include "man_msgs/ExecuteTrajectorySkillAction.h"
#include <webots_ros/BoolStamped.h>

        // boost::mutex mutex; 
        // mutex.lock(); 
        // std::cout << "Thread " << boost::this_thread::get_id()<< std::endl; 
        // mutex.unlock(); 
// #include "man_behavior_tree_nodes/webots_elements.hpp"

namespace man_behavior_tree_nodes
{
class ExecuteTrajectoryActionClient : public btActionClient<man_msgs::ExecuteTrajectorySkillAction, 
                                                        man_msgs::ExecuteTrajectorySkillGoal,
                                                        man_msgs::ExecuteTrajectorySkillResultConstPtr,
                                                        man_msgs::ExecuteTrajectorySkillFeedbackConstPtr,
                                                        webots_ros::BoolStamped::ConstPtr>
{
public:
    ExecuteTrajectoryActionClient(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf,
        float time_for_wait,
        const std::string & subscribe_topic_name);
    
    void initialize();


    void on_tick() override;

    void on_wait_for_result() override;

    void subCallback(webots_ros::BoolStamped::ConstPtr msg) override;

    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<man_msgs::Plan>("plan", ""),             
            BT::OutputPort<int>("result", ""), 
        });
    }

private:
    bool first_time_{true};
    man_msgs::Plan plan_;
    int success_{0};
    bool touch_data_{false};
    std::mutex mtx_sub_;
};
} // namespace

#endif