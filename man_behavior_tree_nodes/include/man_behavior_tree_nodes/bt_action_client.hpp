#ifndef MAN_BEHAVIOR_TREE_NODES_BT_ACTION_CLIENT_HPP_
#define MAN_BEHAVIOR_TREE_NODES_BT_ACTION_CLIENT_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include "actionlib/client/simple_goal_state.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include "man_behavior_tree_nodes/bt_conversions.hpp"


namespace man_behavior_tree_nodes
{
/// The reported execution status
struct ExecutionStatus
{
  enum Value
  {
    UNKNOWN,
    RUNNING,
    SUCCEEDED,
    PREEMPTED,
    TIMED_OUT,
    ABORTED,
    FAILED
  };

  ExecutionStatus(Value value = UNKNOWN) : status_(value)
  {
  }

  operator Value() const
  {
    return status_;
  }

  explicit operator bool() const
  {
    return status_ == SUCCEEDED;
  }

  /// Convert the execution status to a string
  std::string asString() const
  {
    switch (status_)
    {
      case RUNNING:
        return "RUNNING";
      case SUCCEEDED:
        return "SUCCEEDED";
      case PREEMPTED:
        return "PREEMPTED";
      case TIMED_OUT:
        return "TIMED_OUT";
      case ABORTED:
        return "ABORTED";
      case FAILED:
        return "FAILED";
      default:
        return "UNKNOWN";
    }
  }

private:
  Value status_;
};
}

namespace man_behavior_tree_nodes
{

// ActionT moveit_msgs::ExecuteTrajectoryAction
// ActionGoalT moveit_msgs::ExecuteTrajectoryGoal

template<class ActionT, class ActionGoalT, class ActionResultT>
class btActionClient : public BT::ActionNodeBase
{
public:
    btActionClient(
        const std::string & xml_tag_name,
        const std::string & action_name,                     
        const BT::NodeConfiguration & conf,
        float time_for_wait)
    : BT::ActionNodeBase(xml_tag_name, conf), 
    action_name_(action_name),
    wait_for_servers_(ros::WallDuration(time_for_wait))
    {
        // get nodehandle from blackboard
        pnh_ = config().blackboard->get<ros::NodeHandle>("node_handle");
        done_ = true;
        // arm_state_ = config().blackboard->get<robot_state::RobotStatePtr>("arm_state");

        // Initialize the input and output messages
        // goal_state_ =  actionlib::SimpleGoalState::PENDING;

        std::string remapped_action_name;
        if (getInput("server_name", remapped_action_name)) {
        action_name_ = remapped_action_name;
        }



        createActionClient(action_name_);
        last_exec_ = ExecutionStatus::SUCCEEDED;

        // Give the derive class a chance to do any initialization
        ROS_INFO_STREAM_NAMED("btActionClient", xml_tag_name.c_str()<< " btActionClient initialized");
    }

    btActionClient() = delete;

    virtual ~btActionClient()
    {
    }

    void waitForAction(const std::string& name, const ros::WallTime& timeout, double allotted_time)
    {
        ROS_DEBUG_NAMED("btActionClient", "Waiting for move_group action server (%s)...", name.c_str());

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
                ROS_WARN_ONCE_NAMED("btActionClient", "Non-default CallbackQueue: Waiting for external queue "
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
                ROS_WARN_ONCE_NAMED("btActionClient", "Non-default CallbackQueue: Waiting for external queue "
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
            ROS_DEBUG_NAMED("btActionClient", "Connected to '%s'", name.c_str());
        }
    }

    // Create instance of an action server
    void createActionClient(const std::string & action_name)
    {
        timeout_for_servers_ = ros::WallTime::now() + wait_for_servers_;
        if (wait_for_servers_ == ros::WallDuration())
            timeout_for_servers_ = ros::WallTime();  // wait forever
        allotted_time_ = wait_for_servers_.toSec();

        // Now that we have the ROS node to use, create the action client for this BT action
        action_client_.reset(new actionlib::SimpleActionClient<ActionT>(pnh_, action_name_, false));
        waitForAction(action_name_, timeout_for_servers_, allotted_time_);
        // Make sure the server is actually there before continuing        
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
        if (status() == BT::NodeStatus::IDLE) 
        {
        // setting the status to RUNNING to notify the BT Loggers (if any)
            setStatus(BT::NodeStatus::RUNNING);

            // user defined callback
            // define goal
            on_tick();
            
            // send goal
            on_new_goal_received();

        }

        // The following code corresponds to the "RUNNING" loop
        // goal_result_available_ need to be defined in waitforresult
        if (pnh_.ok()  && 
                        (action_client_->getState() == actionlib::SimpleClientGoalState::ACTIVE
                        || action_client_->getState() == actionlib::SimpleClientGoalState::PENDING)
                        ) 
         {
            
            // user defined callback. May modify the value of "goal_updated_"
            on_wait_for_result();

            auto client_status = action_client_->getState();

            // if a new goal coming
            if (goal_updated_ && 
            (client_status == actionlib::SimpleClientGoalState::ACTIVE ||
               client_status == actionlib::SimpleClientGoalState::PENDING)
               )
            {
                 goal_updated_ = false;
                 on_new_goal_received();
            }

            // if collisoion happend
            if (collision_happened_ && 
            (client_status == actionlib::SimpleClientGoalState::ACTIVE ||
               client_status == actionlib::SimpleClientGoalState::PENDING)
               )
            {
                 collision_happened_ = false;
                 collision_happened();
            }


            // // check if, after invoking spin_some(), we finally received the result
            // if (client_status == actionlib::SimpleClientGoalState::ACTIVE ||
            //    client_status == actionlib::SimpleClientGoalState::PENDING) 
            // {
            //     // Yield this Action, returning RUNNING
            //     return BT::NodeStatus::RUNNING;
            // }
        }

        auto client_status = action_client_->getState();

        if (client_status == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            return on_success();
        }

        if (client_status == actionlib::SimpleClientGoalState::ABORTED)
        {
            return on_aborted();
        }

        if (client_status == actionlib::SimpleClientGoalState::PREEMPTED)
        {
            return on_cancelled();
        }
        
        throw std::logic_error("BtActionNode::Tick: invalid status value");
        
    }

    // The other (optional) override required by a BT action. In this case, we
    // make sure to cancel the ROS2 action if it is still running.
    void halt() override
    {
        if (should_cancel_goal()) 
        {
            ROS_INFO_STREAM_NAMED("btActionNode", "Cancelling execution for " << action_name_.c_str());
            action_client_->cancelGoal();
            last_exec_ = ExecutionStatus::PREEMPTED;
            done_ = true;
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

        ros::spinOnce();
        // Check if the goal is still executing
        if (!done_)
        {
            return true;
        }

        return false;  
    }

    bool isConnected() const
    {
        return static_cast<bool>(action_client_);
    }

    ExecutionStatus getLastExecutionStatus()
    {
        return last_exec_;
    }

    void finishControllerExecution(const actionlib::SimpleClientGoalState& state)
    {
        ROS_DEBUG_STREAM_NAMED("btActionNode", "Action " << action_name_ << " is done with state " << state.toString()
                                                                    << ": " << state.getText());
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            last_exec_ = ExecutionStatus::SUCCEEDED;
        else if (state == actionlib::SimpleClientGoalState::ABORTED)
            last_exec_ = ExecutionStatus::ABORTED;
        else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
            last_exec_ = ExecutionStatus::PREEMPTED;
        else
            last_exec_ = ExecutionStatus::FAILED;
        done_ = true;
    }

    BT::NodeStatus collision_happened()
    {
        action_client_->cancelGoal();
        ROS_DEBUG_STREAM_NAMED("btActionNode", "collision_happened, will aborted ");
        return on_aborted();
    }

//      bool cancelExecution() override
//   {
//     if (!controller_action_client_)
//       return false;
//     if (!done_)
//     {
//       ROS_INFO_STREAM_NAMED("ActionBasedController", "Cancelling execution for " << name_);
//       controller_action_client_->cancelGoal();
//       last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
//       done_ = true;
//     }
//     return true;
//   }


    void on_new_goal_received()
    {
        goal_result_available_ = false;
        action_client_->sendGoal(goal_);

        goal_result_available_ = true;


        auto client_status = action_client_->getState();
        if (client_status == actionlib::SimpleClientGoalState::REJECTED)
        {
            throw std::runtime_error("Goal was rejected by the action server");
        }

        if (!action_client_->waitForResult())
        {
            ROS_INFO_STREAM_NAMED("btActionNode", action_name_.c_str()<< "MoveGroup action returned early");
        }
        else
        {
            result_ = action_client_->getResult();
        }
        
    }


        std::string action_name_;
        ros::WallDuration wait_for_servers_;
        typename std::unique_ptr<actionlib::SimpleActionClient<ActionT> > action_client_;

        ActionGoalT goal_;
        ActionResultT result_;

        // actionlib::SimpleGoalState goal_state_;
        // std::unique_ptr<actionlib::SimpleClientGoalState> action_state_;
        // bool goal_updated_{false};
        bool goal_result_available_{false};

        ros::WallTime timeout_for_servers_;
        double allotted_time_;

    // The node that will be used for any ROS operations
        ros::NodeHandle pnh_;
        // robot_state::RobotStatePtr arm_state_;

        bool done_;

        bool goal_updated_{false};

        bool collision_happened_{false};

        ExecutionStatus last_exec_;
};

}  // namespace nav2_behavior_tree

#endif  // MAN_BEHAVIOR_TREE_NODES_BT_ACTION_CLIENT_HPP_
