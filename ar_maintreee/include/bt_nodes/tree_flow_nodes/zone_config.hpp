#ifndef ZONE_CONFIG_HPP
#define ZONE_CONFIG_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <std_msgs/msg/u_int8.hpp>

class ZoneConfig : public BT::SyncActionNode
{
public:
    ZoneConfig(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node_ptr);


    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();


private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr zone_data;    
    void zone_data_subscriber_callback(std_msgs::msg::UInt8 msg);
   
    std::string zone ;
};

#endif