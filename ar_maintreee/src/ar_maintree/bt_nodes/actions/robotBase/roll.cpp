#include "bt_nodes/actions/robotBase/roll.hpp"

#include <chrono>
using namespace std::chrono_literals;

namespace BT
{

Roll::Roll(const std::string &     name,
           const BT::NodeConfig &  config,
           rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config)
, node_(node)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    roll_up_client_ = node_->create_client<motor_interfaces::srv::MotorState>("ak_motor_roll_up");

    prox_front_sub_ = node_->create_subscription<std_msgs::msg::Byte>(
        "/prox_front_received", qos_profile,
        [this](const std_msgs::msg::Byte::SharedPtr msg) {
            front_edge_ = (msg->data == 1);
        });

    prox_rear_sub_ = node_->create_subscription<std_msgs::msg::Byte>(
        "/prox_rear_received", qos_profile,
        [this](const std_msgs::msg::Byte::SharedPtr msg) {
            rear_edge_ = (msg->data == 1);
        });

    if (!node_->has_parameter("roll.roll_up_angle"))
        node_->declare_parameter("roll.roll_up_angle",   10.9);

    if (!node_->has_parameter("roll.roll_down_angle"))
        node_->declare_parameter("roll.roll_down_angle", -11.4);

    RCLCPP_INFO(node_->get_logger(),
                "roll_up_angle = %.2f",
                node_->get_parameter("roll.roll_up_angle").as_double());

    RCLCPP_INFO(node_->get_logger(),
                "roll_down_angle = %.2f",
                node_->get_parameter("roll.roll_down_angle").as_double());
}

BT::PortsList Roll::providedPorts()
{
    return {
        BT::InputPort<int>("climb_state",
            "Climb state: UP=1 or DOWN=0"),
    };
}

BT::NodeStatus Roll::onStart()
{
    reset();

    auto climb = getInput<int>("climb_state");
    if (!climb)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[Roll] Missing required port 'climb_state': %s",
                     climb.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    int climb_state = climb.value();

    if      (climb_state == static_cast<int>(ClimbState::DOWN)) rolling_up_ = true;
    else if (climb_state == static_cast<int>(ClimbState::UP))   rolling_up_ = false;
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "[Roll] Unknown climb_state");
        return BT::NodeStatus::FAILURE;
    }

    roll_up_angle_   = static_cast<float>(
        node_->get_parameter("roll.roll_up_angle").as_double());
    roll_down_angle_ = static_cast<float>(
        node_->get_parameter("roll.roll_down_angle").as_double());

    goal_sent_ = false;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Roll::onRunning()
{
    if (!goal_sent_)
    {
        if (!roll_up_client_->wait_for_service(0s))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "[Roll] Waiting for service 'ak_motor_roll_up'...");
            return BT::NodeStatus::RUNNING;
        }

        auto request   = std::make_shared<motor_interfaces::srv::MotorState::Request>();
        request->data  = true;
        request->state = rolling_up_ ? roll_up_angle_ : roll_down_angle_;

        RCLCPP_INFO(node_->get_logger(),
                    "[Roll] Sending command: ak_motor_roll_up  angle=%.2f  (rolling_up=%s)",
                    request->state, rolling_up_ ? "true" : "false");

        roll_up_client_->async_send_request(
            request,
            [this](rclcpp::Client<motor_interfaces::srv::MotorState>::SharedFuture future)
            {
                auto response = future.get();
                if (response->success)
                    RCLCPP_INFO(node_->get_logger(),
                                "[Roll] Service call succeeded: %s",
                                response->message.c_str());
                else
                    RCLCPP_WARN(node_->get_logger(),
                                "[Roll] Service call failed: %s",
                                response->message.c_str());

                std::lock_guard<std::mutex> lock(result_mutex_);
                call_succeeded_    = response->success;
                response_received_ = true;
            });

        goal_sent_ = true;
        return BT::NodeStatus::RUNNING;
    }

    bool prox_stop = false;

    if (rolling_up_)
    {
        RCLCPP_INFO(node_->get_logger(), "[Roll] roll_up sent — returning SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
    else if (!rolling_up_ && rear_edge_)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[Roll] Rear edge detected — stopping roll_down");
        prox_stop = true;
    }

    if (prox_stop)
        return BT::NodeStatus::SUCCESS;

    return BT::NodeStatus::RUNNING;
}

void Roll::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "[Roll] Halted");

    std::lock_guard<std::mutex> lock(result_mutex_);
    response_received_ = false;
    call_succeeded_    = false;
    goal_sent_          = false;
}

void Roll::reset()
{
    rolling_up_        = false;
    response_received_ = false;
    call_succeeded_    = false;
    goal_sent_          = false;
}

} // namespace BT