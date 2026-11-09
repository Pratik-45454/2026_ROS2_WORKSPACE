#ifndef RESET_HPP_
#define RESET_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "qos_profiles.hpp"

class Reset : public BT::StatefulActionNode
{
public:
    Reset(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::uint8_t>("reset_byte", 0, "Byte value to publish on reset_state")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void publish_reset_byte();
    void publish_zero_twist();

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr reset_byte_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr emergency_stop_client_;

    std::shared_future<std_srvs::srv::SetBool::Response::SharedPtr> result_future_;
    bool request_sent_;
    bool ran_once_;
};

#endif  