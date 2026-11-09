#ifndef INITIALIZE_PORTS_ACTUATORS_HPP
#define INITIALIZE_PORTS_ACTUATORS_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "qos_profiles.hpp"
#include "bt_nodes/tree_flow_nodes/flag_alter.hpp"
#include "bt_nodes/camera_nodes/get_roi_detections.hpp"

class InitializePortsActuators : public BT::StatefulActionNode
{
public:
    InitializePortsActuators(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr team_color_;
   
    std::int8_t set_reset_val_;
    
    MartialStageDetections detections_;

    std::string color_;
    void color_data_subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg);
 
};

#endif