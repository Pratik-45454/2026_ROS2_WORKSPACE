#include "bt_nodes/actions/robotBase/rotate.hpp"
#include <cmath>

namespace BT
{

Rotate::Rotate(const std::string &     name,
               const BT::NodeConfig &  config,
               rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config)
, node_(node)
{
    action_client_ = rclcpp_action::create_client<RotateExec>(node_, "rotate_exec");
}

BT::PortsList Rotate::providedPorts()
{
    return {
        BT::InputPort<float>("rotate_state",
            "Target rotation in DEGREES from DecisionNode"),
        BT::OutputPort<float>("final_yaw",
            "Actual yaw reading when rotation completed"),
    };
}

BT::NodeStatus Rotate::onStart()
{
    reset();

    if (!getInput("rotate_state", target_degrees_))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[Rotate] Missing required port 'rotate_state'");
        return BT::NodeStatus::FAILURE;
    }

    target_angle_ = target_degrees_ * M_PI / 180.0f;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Rotate::onRunning()
{
    if (!goal_sent_)
    {
        if (!action_client_->wait_for_action_server(std::chrono::milliseconds(0)))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "[Rotate] Waiting for rotate_exec server...");
            return BT::NodeStatus::RUNNING;
        }

        RotateExec::Goal goal;
        goal.target_angle = target_angle_;

        auto options = rclcpp_action::Client<RotateExec>::SendGoalOptions{};

        options.goal_response_callback =
            [this](const GoalHandleRotate::SharedPtr & handle) {
                if (!handle) {
                    RCLCPP_ERROR(node_->get_logger(), "[Rotate] Goal REJECTED");
                    goal_rejected_ = true;
                } else {
                    RCLCPP_INFO(node_->get_logger(), "[Rotate] Goal ACCEPTED");
                    goal_handle_   = handle;
                    goal_accepted_ = true;
                }
            };

        options.feedback_callback =
            [this](GoalHandleRotate::SharedPtr,
                   const std::shared_ptr<const RotateExec::Feedback> fb) {
                RCLCPP_DEBUG(node_->get_logger(),
                             "[Rotate] yaw_error=%.4f rad  current_yaw=%.4f rad",
                             fb->current_yaw_error, fb->current_yaw);
            };

        options.result_callback =
            [this](const GoalHandleRotate::WrappedResult & result) {
                goal_done_      = true;
                goal_succeeded_ = result.result->success;
                final_yaw_      = result.result->final_yaw;
                RCLCPP_INFO(node_->get_logger(),
                            "[Rotate] Result: success=%s  final_yaw=%.4f rad",
                            goal_succeeded_ ? "true" : "false", final_yaw_);
            };

        action_client_->async_send_goal(goal, options);
        goal_sent_ = true;

        RCLCPP_INFO(node_->get_logger(),
                    "[Rotate] Goal sent - target_degrees=%.1f  target_rad=%.4f",
                    target_degrees_, target_angle_);
        return BT::NodeStatus::RUNNING;
    }

    if (goal_rejected_)
        return BT::NodeStatus::FAILURE;

    if (goal_done_)
    {
        setOutput("final_yaw", final_yaw_);
        return goal_succeeded_ ? BT::NodeStatus::SUCCESS
                               : BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void Rotate::onHalted()
{
    if (goal_handle_)
    {
        action_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
    }
    RCLCPP_WARN(node_->get_logger(), "[Rotate] Halted - goal cancelled.");
}

void Rotate::reset()
{
    goal_sent_      = false;
    goal_accepted_  = false;
    goal_rejected_  = false;
    goal_done_      = false;
    goal_succeeded_ = false;
    final_yaw_      = 0.0f;
    goal_handle_.reset();
}

} // namespace BT