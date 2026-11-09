#include "bt_nodes/conditions/incremental_scroll.hpp"

IncrementalScroll::IncrementalScroll(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node_(node)
{}

BT::PortsList IncrementalScroll::providedPorts()
{
    return {
        BT::InputPort<int>("action"),
        BT::OutputPort<int>("count"),
        // BT::InputPort<int>("decrease"),
    };
}

BT::NodeStatus IncrementalScroll::tick()
{
    
    auto action = getInput<int>("action");
    // auto decrease     = getInput<int>("decrease");

    action  = action.value();
    // decrease = decrease.value();

    if(action == 0)
        count += 1;
    else if(action == 1)
        count -= 1;

    setOutput<int>("count", count);

    RCLCPP_INFO(node_->get_logger(),
        "[IncrementScrollCount] forest_scrolls_picked = %d", count);
    return BT::NodeStatus::SUCCESS;
}