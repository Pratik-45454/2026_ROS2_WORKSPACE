#include "bt_nodes/tree_flow_nodes/reset.hpp"
#include <chrono>

using namespace std::chrono_literals;

Reset::Reset(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node), request_sent_(false), ran_once_(false)
{
    reset_byte_pub_ = node_->create_publisher<std_msgs::msg::UInt8>(
        "reset_state", 10);

    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", cmd_vel_qos());

    emergency_stop_client_ = node_->create_client<std_srvs::srv::SetBool>(
        "emergency_stop");
}

void Reset::publish_reset_byte()
{
    std::uint8_t reset_byte = 0;
    getInput("reset_byte", reset_byte);

    std_msgs::msg::UInt8 msg;
    msg.data = reset_byte;
    reset_byte_pub_->publish(msg);
}

void Reset::publish_zero_twist()
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    cmd_vel_pub_->publish(msg);
}

BT::NodeStatus Reset::onStart()
{

    if(ran_once_){
        return BT::NodeStatus::FAILURE;
    }
    request_sent_ = false;

    publish_reset_byte();
    publish_zero_twist();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Reset::onRunning()
{
    auto blackboard = config().blackboard;
    blackboard->set<int>("photo_stage_1", 0);

    if(ran_once_){
        return BT::NodeStatus::FAILURE;
    }

    if (!request_sent_) {
        if (!emergency_stop_client_->wait_for_service(0s)) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "[Reset] waiting for emergency_stop service...");
            return BT::NodeStatus::RUNNING;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        result_future_ = emergency_stop_client_->async_send_request(
            request,
            [](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture) {}
        ).future;

        request_sent_ = true;
        return BT::NodeStatus::RUNNING;
    }

    if (result_future_.wait_for(0s) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
    }

    auto response = result_future_.get();
    request_sent_ = false;

    if (response->success) {
        RCLCPP_INFO(node_->get_logger(), "[Reset] emergency_stop succeeded: %s",
            response->message.c_str());

        ran_once_ = true;
        return BT::NodeStatus::FAILURE; //we want failure, in this node failure = success
    }

    RCLCPP_WARN(node_->get_logger(), "[Reset] emergency_stop failed: %s",
        response->message.c_str());
    return BT::NodeStatus::RUNNING; //Keep Trying
}

void Reset::onHalted()
{
    ran_once_ = false;
    request_sent_ = false;
}