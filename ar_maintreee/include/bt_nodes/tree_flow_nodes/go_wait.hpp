#ifndef GO_WAIT_HPP
#define GO_WAIT_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <std_msgs/msg/u_int8.hpp>

class GoWait : public BT::SyncActionNode
{
public:
    GoWait(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node_ptr);


    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr go_wait;    
    void go_wait_subscriber_callback(std_msgs::msg::UInt8 msg);
   
    bool go = false;
};

#endif