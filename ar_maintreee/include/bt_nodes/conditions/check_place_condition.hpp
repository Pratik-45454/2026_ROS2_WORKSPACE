#ifndef CHECK_PLACE_CONDITION_H
#define CHECK_PLACE_CONDITION_H

#include <string>
#include <memory>
#include <std_msgs/msg/u_int8.hpp>



#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"

#include <behaviortree_cpp/bt_factory.h>



class CheckPlaceCondition: public BT::SyncActionNode
{
    public:
    CheckPlaceCondition(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node);

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8>> subscription_;
    
    bool clear;

    BT::NodeStatus tick() override;

   
    void subscriber_callback(std_msgs::msg::UInt8 msg);
    
};


#endif