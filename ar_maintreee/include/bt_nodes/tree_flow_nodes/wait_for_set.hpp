#ifndef WAIT_FOR_SET_HPP
#define WAIT_FOR_SET_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "bt_nodes/camera_nodes/get_roi_detections.hpp"
#include "bt_nodes/tree_flow_nodes/flag_alter.hpp"


class WaitForSet : public BT::SyncActionNode
{
public:
    WaitForSet(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_ptr_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr set_reset;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr trust_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr pick_dropped_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr second_row_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr third_row_sub;

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr forest_blocks_display;

    void set_reset_subscriber_callback(std_msgs::msg::UInt8 msg);
    void trust_callback(std_msgs::msg::UInt8 msg);
    void forest_blocks_callback(std_msgs::msg::UInt8MultiArray msg);

    void pick_dropped_callback(std_msgs::msg::UInt8 msg);
    void second_row_callback(std_msgs::msg::UInt8 msg);
    void third_row_callback(std_msgs::msg::UInt8 msg);

    bool wait  = true;
    bool start = false;

    uint8_t trust_value_ = 0; // last received trust byte
    uint8_t pick_dropped_ = 0; // Change to 0
    uint8_t second_row_ = 0; // last received second_row byte
    uint8_t third_row_ = 0; // last received third_row byte
    std::string row = "";
    
    std::vector<float> SICK_TICTAC_MIDDLE;
    std::vector<float> SICK_TICTAC_LEFT;
    std::vector<float> SICK_TICTAC_RIGHT;   

    bool forest_blocks_received_ = false;  // true once forest_blocks has delivered at least one valid 12-byte array

    MartialStageDetections stage1_;
    MartialStageDetections stage2_;
    MartialStageDetections stage3_;
    MartialStageDetections stage4_;

    int in_martial = 0;
    int spear_assembled = 0;
    int spear_gripped = 0;
    
    int scroll_picked_from_entrance = 0;
    int climb_position_reached = 0;
    int pick_from_entrance = 0;
    int store = 0;
    int carry = 0;
    int climbed_three_photos = 0;

    int in_forest = 0;
    int photo_sequence = 0;

    int in_arena = 0;
    int placed_1 = 0;
    int placed_2 = 0;
    int nav_to_rack = 0;
};

#endif