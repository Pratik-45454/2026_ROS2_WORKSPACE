#ifndef START_WAIT_HPP
#define START_WAIT_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <std_msgs/msg/u_int8.hpp>

class StartWait : public BT::SyncActionNode
{
public:
    StartWait(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node_ptr);

   

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_ptr_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr set_reset;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr init_robot;
   
    void set_reset_subscriber_callback(std_msgs::msg::UInt8 msg);
    void init_robot_subscriber_callback(std_msgs::msg::UInt8 msg);

    uint8_t prev_set_data{0};

    
   
    bool start = false;
};

#endif