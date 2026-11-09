#ifndef CHECK_SECOND_ROW_H
#define CHECK_SECOND_ROW_H

#include <string>
#include <memory>
#include <std_msgs/msg/u_int8.hpp>



#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"

#include <behaviortree_cpp/bt_factory.h>



class CheckSecondRow: public BT::SyncActionNode
{
    public:
    CheckSecondRow(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node);

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8>> subscription_;
    
    bool second;

    BT::NodeStatus tick() override;

    void subscriber_callback(std_msgs::msg::UInt8 msg);
    
};


#endif