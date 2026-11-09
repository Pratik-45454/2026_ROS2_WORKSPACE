#pragma once
#include <string>
#include <chrono>
#include <cstdint>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/byte.hpp>



class InitRobot : public BT::StatefulActionNode
{
public:
    InitRobot(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void color_data_subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void set_bit_and_publish(uint8_t bit);
    void clear_bit_and_publish(uint8_t bit);

    rclcpp::Node::SharedPtr node_ptr_;

    rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr activate_motors;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr team_color_;

    std::string color_{""};

    uint8_t motor_bits_{0};
    bool    bit_set_{false};
    long    start_time_{0};
};