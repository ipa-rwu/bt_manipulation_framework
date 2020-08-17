#ifndef MAN_BEHAVIOR_TREE_NODES__BT_ACTION_CLIENT_HPP_
#define MAN_BEHAVIOR_TREE_NODES__BT_ACTION_CLIENT_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>

namespace man_behavior_tree_nodes
{

// ActionT moveit_msgs::ExecuteTrajectoryAction
// ActionGoalT moveit_msgs::ExecuteTrajectoryGoal

template<class ActionT, class ActionGoalT>
class btActionNode : public BT::ActionNodeBase
{
public:
    btActionNode(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const ros::WallDuration& wait_for_servers,                        //action server name
        const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf), 
    action_name_(action_name), 
    wait_for_servers_(wait_for_servers)
    {
        // get nodehandle from blackboard
        pnh_ = config().blackboard->get<ros::NodeHandle>("private_node_handle");

        // Initialize the input and output messages
        goal_ = typename ActionGoalT();
        action_state_ =  actionlib::SimpleClientGoalState::PENDING;

        std::string remapped_action_name;
        if (getInput("server_name", remapped_action_name)) {
        action_name_ = remapped_action_name;
        }

        timeout_for_servers_ = ros::WallTime::now() + wait_for_servers_;
        if (wait_for_servers_ == ros::WallDuration())
            timeout_for_servers = ros::WallTime();  // wait for ever
        allotted_time_ = wait_for_servers_.toSec();

        createActionClient(action_name_);

        // Give the derive class a chance to do any initialization
        RCLCPP_INFO(node_->get_logger(), "\"%s\" BtActionNode initialized", xml_tag_name.c_str());
    }

    BtActionNode() = delete;

    virtual ~BtActionNode()
    {
    }

    void waitForAction(const std::string& name, const ros::WallTime& timeout, double allotted_time)
  {
    ROS_DEBUG_NAMED(LOGNAME, "Waiting for move_group action server (%s)...", name.c_str());

    // wait for the server (and spin as needed)
    if (timeout == ros::WallTime())  // wait forever
    {
      while (pnh_.ok() && !action_client_->isServerConnected())
      {
        ros::WallDuration(0.001).sleep();
        // explicit ros::spinOnce on the callback queue used by NodeHandle that manages the action client
        ros::CallbackQueue* queue = dynamic_cast<ros::CallbackQueue*>(pnh_.getCallbackQueue());
        if (queue)
        {
          queue->callAvailable();
        }
        else  // in case of nodelets and specific callback queue implementations
        {
          ROS_WARN_ONCE_NAMED(LOGNAME, "Non-default CallbackQueue: Waiting for external queue "
                                       "handling.");
        }
      }
    }
    else  // wait with timeout
    {
      while (pnh_.ok() && !action_client_->isServerConnected() && timeout > ros::WallTime::now())
      {
        ros::WallDuration(0.001).sleep();
        // explicit ros::spinOnce on the callback queue used by NodeHandle that manages the action client
        ros::CallbackQueue* queue = dynamic_cast<ros::CallbackQueue*>(pnh_.getCallbackQueue());
        if (queue)
        {
          queue->callAvailable();
        }
        else  // in case of nodelets and specific callback queue implementations
        {
          ROS_WARN_ONCE_NAMED(LOGNAME, "Non-default CallbackQueue: Waiting for external queue "
                                       "handling.");
        }
      }
    }

    if (!action_client_->isServerConnected())
    {
      std::stringstream error;
      error << "Unable to connect to move_group action server '" << name << "' within allotted time (" << allotted_time
            << "s)";
      throw std::runtime_error(error.str());
    }
    else
    {
      ROS_DEBUG_NAMED(LOGNAME, "Connected to '%s'", name.c_str());
    }
  }

    // Create instance of an action server
    void createActionClient(const std::string & action_name)
    {
        // Now that we have the ROS node to use, create the action client for this BT action
        action_client_.reset(new actionlib::SimpleActionClient<ActionT>(
            pnh_, action_name_, false));

        // Make sure the server is actually there before continuing
        ROS_INFO("Waiting for \"%s\" action server", action_name.c_str());
        waitForAction(action_name_, timeout_for_servers_, allotted_time_);
    }

    // Any subclass of BtActionNode that accepts parameters must provide a providedPorts method
    // and call providedBasicPorts in it.
    static BT::PortsList providedBasicPorts(BT::PortsList addition)
    {
        BT::PortsList basic = {
        BT::InputPort<std::string>("server_name", "Action server name"),
        BT::InputPort<std::chrono::milliseconds>("server_timeout")
        };
        basic.insert(addition.begin(), addition.end());

        return basic;
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({});
    }

    // Derived classes can override any of the following methods to hook into the
    // processing for the action: on_tick, on_wait_for_result, and on_success

    // Could do dynamic checks, such as getting updates to values on the blackboard
    virtual void on_tick()
    {
    }

    // There can be many loop iterations per tick. Any opportunity to do something after
    // a timeout waiting for a result that hasn't been received yet
    virtual void on_wait_for_result()
    {
    }

    // Called upon successful completion of the action. A derived class can override this
    // method to put a value on the blackboard, for example.
    virtual BT::NodeStatus on_success()
    {
        return BT::NodeStatus::SUCCESS;
    }

    // Called when a the action is aborted. By default, the node will return FAILURE.
    // The user may override it to return another value, instead.
    virtual BT::NodeStatus on_aborted()
    {
        return BT::NodeStatus::FAILURE;
    }

    // Called when a the action is cancelled. By default, the node will return SUCCESS.
    // The user may override it to return another value, instead.
    virtual BT::NodeStatus on_cancelled()
    {
        return BT::NodeStatus::SUCCESS;
    }

    // The main override required by a BT action
    BT::NodeStatus tick() override
    {
        // first step to be done only at the beginning of the Action
        if (status() == BT::NodeStatus::IDLE) {
        // setting the status to RUNNING to notify the BT Loggers (if any)
        setStatus(BT::NodeStatus::RUNNING);

        // user defined callback
        on_tick();

        on_new_goal_received();


        }

        // The following code corresponds to the "RUNNING" loop
        if (pnh_.ok()  && !goal_result_available_) {
        // user defined callback. May modify the value of "goal_updated_"
        on_wait_for_result();

        auto goal_status = action_client_->getState();
        // if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
        //     goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
        // {
        //     goal_updated_ = false;
        //     on_new_goal_received();
        // }


        // check if, after invoking spin_some(), we finally received the result
        if (!goal_result_available_) {
            // Yield this Action, returning RUNNING
            return BT::NodeStatus::RUNNING;
        }
        }

        if (action_state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            return on_success();
        }

        if (action_state_ == actionlib::SimpleClientGoalState::ABORTED)
        {
            return on_aborted();
        }

        if (action_state_ == actionlib::SimpleClientGoalState::CANCELED)
        {
            return on_cancelled();
        }
        
        throw std::logic_error("BtActionNode::Tick: invalid status value");
        
    }

    // The other (optional) override required by a BT action. In this case, we
    // make sure to cancel the ROS2 action if it is still running.
    void halt() override
    {
        if (should_cancel_goal()) {
        auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
        if (rclcpp::spin_until_future_complete(node_, future_cancel) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(
            node_->get_logger(),
            "Failed to cancel action server for %s", action_name_.c_str());
        }
        }

        setStatus(BT::NodeStatus::IDLE);
    }

protected:
    bool should_cancel_goal()
    {
        // Shut the node down if it is currently running
        if (status() != BT::NodeStatus::RUNNING) {
        return false;
        }

        rclcpp::spin_some(node_);
        auto status = goal_handle_->get_status();

        // Check if the goal is still executing
        return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
            status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
    }


    void on_new_goal_received()
    {
        goal_result_available_ = false;
        action_client_->sendgoal(goal_);

        goal_result_available_ = true;

        if (!action_client_->waitForResult())
        {
        ROS_INFO_STREAM_NAMED(action_name_.c_str(), "MoveGroup action returned early");
        }
        action_state_ = action_client_->getState();
    }


        std::string action_name_;
        ros::WallDuration wait_for_servers_;
        typename std::unique_ptr<actionlib::SimpleActionClient<ActionT> > action_client_;

        typename ActionGoalT goal_;
        actionlib::SimpleClientGoalState action_state_;
        // bool goal_updated_{false};
        bool goal_result_available_{false};

        ros::WallTime timeout_for_servers_;
        double allotted_time_;
    // The node that will be used for any ROS operations
        ros::NodeHandle pnh_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
