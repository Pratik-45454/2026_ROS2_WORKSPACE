#include "bt_nodes/tree_flow_nodes/wait_for_lift.hpp"

WaitForLift::WaitForLift(const std::string &name, const BT::NodeConfig &config,
                                  rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    prox_rear_sub_ = node_->create_subscription<std_msgs::msg::Byte>(
        "prox_rear_received", qos_profile,
        std::bind(&WaitForLift::prox_rear_callback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "WaitForLift initialized");
}

void WaitForLift::prox_rear_callback(std_msgs::msg::Byte::SharedPtr msg)
{
    prox_rear_val_ = msg->data;
}

BT::NodeStatus WaitForLift::onStart()
{
    return onRunning();
}

BT::NodeStatus WaitForLift::onRunning()
{
    RCLCPP_INFO(node_->get_logger(), "WaitForLift RUNNING");
    if (prox_rear_val_.load() == LIFTED_VALUE)
        return BT::NodeStatus::SUCCESS;

    return BT::NodeStatus::RUNNING;
}

void WaitForLift::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "WaitForLift halted");
}