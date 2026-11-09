#pragma once
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte.hpp"
#include <atomic>

class WaitForLift : public BT::StatefulActionNode
{
public:
    WaitForLift(const std::string &name, const BT::NodeConfig &config,
                     rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void prox_rear_callback(std_msgs::msg::Byte::SharedPtr msg);

    static constexpr uint8_t LIFTED_VALUE = 1;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr prox_rear_sub_;
    std::atomic<uint8_t> prox_rear_val_{255};
};