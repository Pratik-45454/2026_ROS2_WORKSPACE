#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ar_interfaces_2026/action/align_arena_twist.hpp"

using AlignArenaTwist = ar_interfaces_2026::action::AlignArenaTwist;
using GoalHandleAlignArena = rclcpp_action::ClientGoalHandle<AlignArenaTwist>;

class AlignArena : public BT::StatefulActionNode
{
public:
    AlignArena(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void reset();

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<AlignArenaTwist>::SharedPtr action_client_;
    GoalHandleAlignArena::SharedPtr goal_handle_;

    float target_x_{0.0f};
    float target_y_{0.0f};

    bool goal_sent_{false};
    bool goal_rejected_{false};
    bool goal_done_{false};
    bool goal_succeeded_{false};
};