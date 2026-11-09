
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "bt_nodes/conditions/check_for_fallen_scroll.hpp"
#include "behaviortree_cpp/bt_factory.h"


using namespace BT;
using namespace std::placeholders;

CheckForFallenScroll::CheckForFallenScroll(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node
):BT::SyncActionNode(name, config),node_(node)
{
}

BT::PortsList CheckForFallenScroll::providedPorts()
{
    return{
        BT::InputPort<uint8_t>("pick_dropped")
    };
}
BT::NodeStatus CheckForFallenScroll::tick()
{
    auto pick_dropped = getInput<uint8_t>("pick_dropped").value();
    if(pick_dropped == 1)
    {
        RCLCPP_INFO(node_->get_logger(),"[CheckForFallenScroll]:: yes");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}
