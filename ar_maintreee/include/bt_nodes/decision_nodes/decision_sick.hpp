// decision_sick.hpp
#ifndef DECISION_SICK_HPP_
#define DECISION_SICK_HPP_
#pragma once

#include <string>
#include <memory>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "bt_nodes/camera_nodes/get_roi_detections.hpp"

#include "bt_nodes/actions/robotBase/yaw_setpoint.hpp"


class DecisionSick : public BT::SyncActionNode
{
public:
    DecisionSick(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    void set_setpoints(const MartialStageDetections &det, const int yaw_int_,
                       std::string &turn_output, int &store, int &climb_position_reached, int &pick_from_entrance, std::string color_);

    std::string under_roi;
    int climb_position_reached{0};
    int pick_from_entrance{0};
    std::vector<float>true_setpoint_;
    std::vector<float> grip_setpoint_;

    std::vector<float> SICK_SP_MIDDLE;

    std::vector<float> SICK_SP_NEAR;
    std::vector<float> SICK_SP_NEAR_GRIP;

    std::vector<float> SICK_SP_FAR;
    std::vector<float> SICK_SP_FAR_GRIP;
};
#endif // DECISION_SICK_HPP_