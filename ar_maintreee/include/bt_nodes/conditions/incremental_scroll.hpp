#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"

class IncrementalScroll : public BT::SyncActionNode
{
public:
    IncrementalScroll(
        const std::string & name,
        const BT::NodeConfig & config,
        rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    int count = 0;
};