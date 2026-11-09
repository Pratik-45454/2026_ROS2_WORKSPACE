#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"



class DecisionGripperHeight : public BT::SyncActionNode
{
public:
    DecisionGripperHeight(
        const std::string &name,
        const BT::NodeConfig &config,
        const rclcpp::Node::SharedPtr &node
    );
     
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    rclcpp::Node::SharedPtr node_;

    

};



