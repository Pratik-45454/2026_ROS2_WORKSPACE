#include "bt_nodes/conditions/check_goal_node.hpp"

CheckGoalNode::CheckGoalNode(const std::string & name,
                              const BT::NodeConfig & config,
                              const rclcpp::Node::SharedPtr & node)
: BT::ConditionNode(name, config)
, node_(node)
{
}

BT::PortsList CheckGoalNode::providedPorts()
{
    return {
        BT::InputPort<int>("goal_node"),
        BT::InputPort<int>("value")
    };
}

BT::NodeStatus CheckGoalNode::tick()
{
    int goal_node = 0, value = 0;

    if (!getInput("goal_node", goal_node))
    {
        throw BT::RuntimeError("[CheckGoalNode] Missing required input 'goal_node'");
    }
    if (!getInput("value", value))
    {
        throw BT::RuntimeError("[CheckGoalNode] Missing required input 'value'");
    }

    return (goal_node == value) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}