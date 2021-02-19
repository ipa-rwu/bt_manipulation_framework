#ifndef MAN_BEHAVIOR_TREE_NODES_BT_ACTION_CLIENT_SUBSCRIBER_HPP_
#define MAN_BEHAVIOR_TREE_NODES_BT_ACTION_CLIENT_SUBSCRIBER_HPP_

#include <memory>
#include <string>
#include <mutex> 
#include <boost/bind.hpp>
#include <boost/thread.hpp>                       
#include <iostream> 

#include "behaviortree_cpp_v3/action_node.h"
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include "actionlib/client/simple_goal_state.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include "man_behavior_tree_nodes/bt_conversions.hpp"

namespace man_behavior_tree_nodes
{
template<class ActionT, class ActionGoalT, class ActionResultT, class ActionFeedbackT, class TopicMsgTypeT>
class btActionClient : public BT::ActionNodeBase
{
public:
    btActionClient(
        const std::string & xml_tag_name,
        const std::string & action_name,                     
        const BT::NodeConfiguration & conf,
        float time_for_wait,
        const std::string & subscribe_topic_name)
    : BT::ActionNodeBase(xml_tag_name, conf), 
        action_name_(action_name),
        wait_for_servers_(ros::WallDuration(time_for_wait)),
        subscribe_topic_name_(subscribe_topic_name)
    {
        // get nodehandle from blackboard
        pnh_ = config().blackboard->get<ros::NodeHandle>("node_handle");
        done_ = true;

        std::string remapped_topic_name;
        if (getInput("topic_name", remapped_topic_name)) {
            subscribe_topic_name_ = remapped_topic_name;
        }

        // if want to redefine action server name
        std::string remapped_action_name;
        if (getInput("server_name", remapped_action_name)) {
            action_name_ = remapped_action_name;
        }

        // if want to redefine timeout for waiting for action result
        double timeout_action_result;
        if (getInput("wait_result_timeout", timeout_action_result)) {
            timeout_action_result_ = ros::Duration(timeout_action_result);
        }
        
        // Give the derive class a chance to do any initialization
        initialzation();

        createActionClient(action_name_);
        #ifdef ENABLE_SUBSCRIBE
            this->createTopicSubscriber();
        #endif

        ROS_INFO_STREAM_NAMED(action_name_+"_client", action_name_+"_client: "<< "initialized");
    }

    btActionClient() = delete;

    virtual ~btActionClient()
    {
    }

    virtual void initialzation()
    {
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
        ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client: " << allotted_time_);
        waitForActionServer(action_name_, timeout_for_servers_, allotted_time_);
        // Make sure the server is actually there before continuing        
    }

    void createTopicSubscriber()
    {
        topic_subscribe_ = pnh_.subscribe(subscribe_topic_name_,
                          1,
                          &btActionClient::subCallback,
                          this);
    }
    
    virtual void subCallback(TopicMsgTypeT msg)
    {
    }

    void waitForActionServer(const std::string& name, const ros::WallTime& timeout, double allotted_time)
    {
        ROS_DEBUG_STREAM_NAMED(name+"_client", name+"_client" <<": Waiting for action server");

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
                    ROS_WARN_STREAM_ONCE_NAMED(name+"_client", name+"_client" <<": Non-default CallbackQueue: Waiting for external queue "
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
                    ROS_WARN_STREAM_ONCE_NAMED(name+"_client", name+"_client" << "Non-default CallbackQueue: Waiting for external queue, handling.");
                }
            }
        }

        if (!action_client_->isServerConnected())
        {
            std::stringstream error;
            ROS_ERROR_STREAM_NAMED(name+"_client", name+"_client" <<": Unable to connect to " << name);
            error << name+"_client: " << "Unable to connect to action server: '" << name << "' within allotted time (" << allotted_time
                    << "s)";
            throw std::runtime_error(error.str());
        }
        else
        {
            ROS_INFO_STREAM_NAMED(name+"_client", name+"_client" <<": Connected to " << name.c_str());
        }
    }

    // Any subclass of BtActionNode that accepts parameters must provide a providedPorts method
    // and call providedBasicPorts in it.
    static BT::PortsList providedBasicPorts(BT::PortsList addition)
    {
        BT::PortsList basic = {
            BT::InputPort<std::string>("server_name", "Action server name"),
            BT::InputPort<std::string>("topic_name", "subscribe topic name"),
            BT::InputPort<std::chrono::milliseconds>("server_timeout"),
            BT::InputPort<double>("wait_result_timeout", "time for waiting for action result"),
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
        mtx_interrupt_.lock();
        interrupt_ = false;
        mtx_interrupt_.unlock();
        mtx_finished_.lock();
        finished_ = false;
        mtx_finished_.unlock();
        return BT::NodeStatus::SUCCESS;
    }

    // Called when a the action is aborted. By default, the node will return FAILURE.
    // The user may override it to return another value, instead.
    virtual BT::NodeStatus on_aborted()
    {
        mtx_interrupt_.lock();
        interrupt_ = false;
        mtx_interrupt_.unlock();
        mtx_finished_.lock();
        finished_ = false;
        mtx_finished_.unlock();
        return BT::NodeStatus::FAILURE;
    }

    // Called when a the action is cancelled. By default, the node will return SUCCESS.
    // The user may override it to return another value, instead.
    virtual BT::NodeStatus on_cancelled()
    {
        mtx_interrupt_.lock();
        interrupt_ = false;
        mtx_interrupt_.unlock();
        mtx_finished_.lock();
        finished_ = false;
        mtx_finished_.unlock();
        return BT::NodeStatus::SUCCESS;
    }

    std::string state_convert(actionlib::SimpleClientGoalState state){
        if(action_client_->getState() == actionlib::SimpleClientGoalState::ACTIVE){
            return ("ACTIVE");
        }
        if(action_client_->getState() == actionlib::SimpleClientGoalState::PENDING){
            return ("PENDING");
        }
        if(action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            return ("SUCCEEDED");
        }
        if(action_client_->getState() == actionlib::SimpleClientGoalState::REJECTED){
            return ("REJECTED");
        }
        if(action_client_->getState() == actionlib::SimpleClientGoalState::ABORTED){
            return ("ABORTED");
        } 
        if(action_client_->getState() == actionlib::SimpleClientGoalState::PREEMPTED){
            return ("PREEMPTED");
        } 
        if(action_client_->getState() == actionlib::SimpleClientGoalState::LOST){
            return ("LOST");
        } 
        else
            return ("UNKNOWN");
    }

    // The main override required by a BT action
    BT::NodeStatus tick() override
    {
        // first step to be done only at the beginning of the Action
        if (status() == BT::NodeStatus::IDLE) 
        {
            // setting the status to RUNNING to notify the BT Loggers (if any)
            setStatus(BT::NodeStatus::RUNNING);
            ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client" <<": Started, action state: " << state_convert(action_client_->getState()));

            // user defined goal, override
            on_tick();
            mtx_finished_.lock();
            finished_ = false;
            mtx_finished_.unlock();
            
            // send goal
            // on_new_goal_received();
            wait_result_thread_.reset(new boost::thread(boost::bind(&btActionClient::on_new_goal_received, this)));
            wait_result_thread_->join();
        }
        
        ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client" <<": After sent, action state: " << state_convert(action_client_->getState()));

        // The following code corresponds to the "RUNNING" loop
        // goal_result_available_ need to be defined in waitforresult
        if (pnh_.ok()){
            // sent goal, pending, maybe cancel before goal available
            if(!finished_){
                if(interrupt_)
                {
                    ROS_WARN_STREAM_NAMED(action_name_+"_client", action_name_+"_client: collisoion happended");
                    wait_result_thread_->interrupt();
                    finished_ = true;
                    return on_aborted();
                }

                // active, waitng for goal
                if(!goal_result_available_){
                    if (action_client_->getState() == actionlib::SimpleClientGoalState::ACTIVE
                        || action_client_->getState() == actionlib::SimpleClientGoalState::PENDING){
                        // user defined callback. May modify the value of "goal_updated_"
                        on_wait_for_result();
                        
                        // if a new goal coming
                        if (goal_updated_)
                        {
                            action_client_->cancelGoal();
                            goal_updated_ = false;
                            wait_result_thread_->interrupt();
                            wait_result_thread_.reset(new boost::thread(boost::bind(&btActionClient::on_new_goal_received, this)));
                            wait_result_thread_->join();
                        }

                        if(interrupt_)
                        {
                            ROS_WARN_STREAM_NAMED(action_name_+"_client", action_name_+"_client: collisoion happended");
                            wait_result_thread_->interrupt();
                            finished_ = true;
                            return on_aborted();
                        }
                    }
                    
                    if(action_client_->getState() == actionlib::SimpleClientGoalState::LOST){
                        return on_aborted();
                    }
                    ros::spinOnce();
                    return BT::NodeStatus::RUNNING;
                }
                else{
                    return on_success();
                }
            }
            else{
                // goal_result_available_
                auto client_status = action_client_->getState();
                if(interrupt_)
                {
                    ROS_WARN_STREAM_NAMED(action_name_+"_client", action_name_+"_client: collisoion happended");
                    wait_result_thread_->interrupt();
                    return on_aborted();
                }
                else{
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

                    if (client_status == actionlib::SimpleClientGoalState::RECALLED)
                    {
                        return on_aborted();
                    }
                    if (client_status == actionlib::SimpleClientGoalState::LOST)
                    {
                        return on_aborted();
                    }
                }
            }
        }
        ROS_ERROR_STREAM_NAMED(action_name_+"_client", action_name_+"_client: BtActionNode::Tick: invalid status value, fix base class");
        return on_aborted();
        // throw std::logic_error("BtActionNode::Tick: invalid status value");        
    }

    void halt() override
    {
        if (should_cancel_goal()) 
        {
            ROS_WARN_STREAM_NAMED(action_name_+"_client", action_name_+"_client: Cancelling execution");
            action_client_->cancelGoal();
            finished_ = true;
        }

        setStatus(BT::NodeStatus::IDLE);
    }
    
    // The user may override it to return another value, instead.
    virtual void feedbackCB(const ActionFeedbackT& feedback){
        ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client: feedbackCB");
    }

    virtual void doneCB(const actionlib::SimpleClientGoalState& state,
            const ActionResultT result)
    {
        finished_ = true;
        goal_result_available_ = true;
        result_ = result;
        ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client: doneCB");
    }

    virtual void activeCB()
    {
        finished_ = false;
        goal_result_available_ = false;
        ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client: activeCB");
    }

    bool interrupt{false};

protected:
    bool should_cancel_goal()                                                                               
    {
        // Shut the node down if it is currently running
        if (status() != BT::NodeStatus::RUNNING) {
        return false;                                                                                                                                                         
        }

        ros::spinOnce();
        // Check if the goal is still executing
        if (!finished_)
        {
            return true;
        }

        return false;  
    }

    bool isConnected() const
    {
        return static_cast<bool>(action_client_);
    }

    BT::NodeStatus collision_happened()
    {
        action_client_->cancelGoal();
        ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client: collision happened, aborted");
        return on_aborted();
    }

    void on_new_goal_received()
    {
        typedef actionlib::SimpleActionClient<ActionT> Client;
        goal_result_available_ = false;
        finished_ = false;
        try{
            if(collision_happened_ == false){
                action_client_->sendGoal(goal_, 
                boost::bind(&btActionClient::doneCB, this, _1, _2),
                boost::bind(&btActionClient::activeCB, this),
                boost::bind(&btActionClient::feedbackCB, this, _1));
                // typename Client::SimpleDoneCallback(),
                // typename Client::SimpleActiveCallback(), 
            }
            else{
                finished_ = true;
            }
        }
        catch (boost::thread_interrupted&) 
        { 
            action_client_->cancelGoal();
        } 
        ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client" <<": send goal, action state: " << state_convert(action_client_->getState()));
    }

    std::string action_name_;
    ros::WallDuration wait_for_servers_;
    typename std::unique_ptr<actionlib::SimpleActionClient<ActionT> > action_client_;
    ActionGoalT goal_;
    ActionResultT result_;
    ros::Subscriber topic_subscribe_;
    // actionlib::SimpleGoalState goal_state_;
    // std::unique_ptr<actionlib::SimpleClientGoalState> action_state_;
    bool goal_result_available_{false};

    ros::WallTime timeout_for_servers_;

    ros::Duration timeout_action_result_{30};
    double allotted_time_;

    // The node that will be used for any ROS operations
    ros::NodeHandle pnh_;
    // robot_state::RobotStatePtr arm_state_;

    bool done_;
    // change in individual function
    bool goal_updated_{false};

    bool collision_happened_{false};

    bool finished_{false};
    bool interrupt_{false};

    std::mutex mtx_interrupt_;
    std::mutex mtx_finished_;


    std::string subscribe_topic_name_{""};

private:
    std::unique_ptr<boost::thread> wait_result_thread_;
};

}  // namespace nav2_behavior_tree

#endif  // MAN_BEHAVIOR_TREE_NODES_BT_ACTION_CLIENT_SUBSCRIBER_HPP_