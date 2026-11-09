#ifndef IS_PATH_CLEAR_H
#define IS_PATH_CLEAR_H

#include <string>
#include <memory>
#include <std_msgs/msg/u_int8.hpp>



#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"

#include <behaviortree_cpp/bt_factory.h>

class isPathClear: public BT::SyncActionNode
{
    public:
    isPathClear(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node_ptr); 

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8>> subscription_;
    
    bool clear;

    BT::NodeStatus tick() override;

    // Subscriber callback
    void subscriber_callback(std_msgs::msg::UInt8 msg);
    
};


#endif