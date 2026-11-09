#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

class CheckGoalNode : public BT::ConditionNode
{
public:
    CheckGoalNode(const std::string & name,
                  const BT::NodeConfig & config,
                  const rclcpp::Node::SharedPtr & node);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
};