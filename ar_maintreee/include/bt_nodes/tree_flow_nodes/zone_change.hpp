#ifndef ZONE_CHANGE_HPP
#define ZONE_CHANGE_HPP

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "qos_profiles.hpp"
#include "bt_nodes/tree_flow_nodes/flag_alter.hpp"

using std::string;

class ZoneChange : public BT::StatefulActionNode
{
public:
    ZoneChange(const string &name, const BT::NodeConfig &config, const rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    // ROS node
    rclcpp::Node::SharedPtr node_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr team_color_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr set_reset_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr reset_to_martial_sub_;

    //For opening up the emergency stop after stopping
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr emergency_stop_client_;
    std::shared_future<std_srvs::srv::SetBool::Response::SharedPtr> emergency_stop_future_;
    bool emergency_stop_request_sent_;
    bool emergency_stop_done_;
    
    // Callbacks
    void set_reset_subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void reset_to_martial_callback(const std_msgs::msg::UInt8::SharedPtr msg);

    // State
    std::string color_;
    std::string state_;
    uint8_t set_reset_val_ = 0x00;   // neutral default (not 0x00 or 0x01)
    uint8_t reset_to_martial_ = 0x00;
    uint8_t prev = 0x00;
    uint8_t current = 0x00; //Vars used to make sure that emergency open is only called during transition from 0 to 1

    int in_martial = 0;
    int spear_assembled = 0;
    int spear_gripped = 0; //Only difference from the spear_assembled is that spear_assembled is set in the "if" of if then else
    
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
    
    enum class ZoneState { NONE, MARTIAL, FOREST, ARENA }; //Used to make sure flags dont get reset AGAIN AND AGAINA in zone change
    ZoneState last_applied_zone_ = ZoneState::NONE;
};
#endif