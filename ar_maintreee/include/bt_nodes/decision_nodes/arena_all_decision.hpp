#pragma once
#include <string>
#include <memory>
#include "behaviortree_cpp/action_node.h"
#include "bt_nodes/camera_nodes/get_roi_detections.hpp"   // for MartialStageDetections


class tree_ticker;

class ArenaAllDecision : public BT::SyncActionNode
{
public:
    
    ArenaAllDecision(
        const std::string & name,
        const BT::NodeConfig & config,
        std::shared_ptr<tree_ticker> /*unused*/ = nullptr);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};