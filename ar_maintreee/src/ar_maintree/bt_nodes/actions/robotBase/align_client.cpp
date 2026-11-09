#include "bt_nodes/actions/robotBase/align_client.hpp"
 
using namespace std::placeholders;
AlignClient::AlignClient(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node) : 
    BT::StatefulActionNode(name, config), node_(node)
{

        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); 

        client_ptr_align = rclcpp_action::create_client<ar_interfaces_2026::action::Bool>(node, "/align_pid"); //movement_pid is the service name
        // sick_subscriber = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        //     "/sick_data", qos_profile,
        //     std::bind(&AlignClient::sickCallback, node_, std::placeholders::_1));



        // odom_subscriber = node_->create_subscription<nav_msgs::msg::Odometry>(
        //     "/odom_data", qos_profile,
        //     std::bind(&AlignClient::odometry_callback, node_, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(),"Align_Front initialized");
        done_flag = false;
}

BT::PortsList AlignClient::providedPorts()
{
    return {};
}

BT::NodeStatus AlignClient::onStart()
{
    auto goal_options = rclcpp_action::Client<ar_interfaces_2026::action::Bool>::SendGoalOptions();
    goal_options.goal_response_callback = std::bind(&AlignClient::goal_response_callback, this, _1); //When server sends a response to a goal request, run node_ callback
    goal_options.feedback_callback = std::bind(&AlignClient::feedback_callback, this, _1, _2);
    goal_options.result_callback = std::bind(&AlignClient::result_callback, this, _1);

    //goal_msg.setpoint_sick = std::vector<float>{0.0f, 0.0f, 0.0f, 0.0f};
    goal_msg.data = true;

    start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::steady_clock::now().time_since_epoch())
                 .count();
    
    if (!client_ptr_align->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
        done_flag = true;
        success = false;
        return BT::NodeStatus::FAILURE;
    }

    client_ptr_align->async_send_goal(goal_msg, goal_options);
    RCLCPP_INFO(node_->get_logger(), "AlignClient::sent goal");
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignClient::onRunning()
    {
    uint32_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::steady_clock::now().time_since_epoch())
                 .count();

    if (now - start_time >= 3000)
    {
        if (done_flag && success)
        {
            return BT::NodeStatus::SUCCESS; 
        }

        if (done_flag && !success)
        {
            return BT::NodeStatus::FAILURE;
        }
    }
    
    return BT::NodeStatus::RUNNING;
}

void AlignClient::onHalted()
{
    cancel_goal();  
}

void AlignClient::cancel_goal()
{
    if (goal_handle)
    {
        client_ptr_align->async_cancel_goal(goal_handle);
        RCLCPP_INFO(node_->get_logger(), "AlignClient::Goal canceled");
    }
}

void AlignClient::goal_response_callback(const GoalHandleMovement::SharedPtr &goal_handle_)
{

    if (!goal_handle_)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "AlignClient::Goal rejected by server");
        done_flag = true;
        success = false;
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(),
                    "AlignClient::Goal accepted");
        goal_handle = goal_handle_;
    }

}

void AlignClient::result_callback(const GoalHandleMovement::WrappedResult &result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "AlignClient::Goal succeeded");
        success = true;
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(),
                    "AlignClient::Goal failed or canceled");
        success = false;
    }
    done_flag = true;
}

void AlignClient::feedback_callback(
    GoalHandleMovement::SharedPtr,
    const std::shared_ptr<const ar_interfaces_2026::action::Bool::Feedback> feedback)
{
    (void)feedback;

}

