#include "bt_nodes/actions/robotBase/align_arena.hpp"

AlignArena::AlignArena(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config)
, node_(node)
{
    action_client_ = rclcpp_action::create_client<AlignArenaTwist>(node_, "align_arena_twist");
}

BT::PortsList AlignArena::providedPorts()
{
    return {
        BT::InputPort<float>("target_x"),
        BT::InputPort<float>("target_y"),
    };
}

BT::NodeStatus AlignArena::onStart()
{
    reset();
    getInput("target_x", target_x_);
    getInput("target_y", target_y_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignArena::onRunning()
{
    if (!goal_sent_)
    {
        if (!action_client_->wait_for_action_server(std::chrono::milliseconds(0)))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "[AlignArena] Waiting for align_arena_twist server...");
            return BT::NodeStatus::RUNNING;
        }

        AlignArenaTwist::Goal goal;
        goal.target_x = target_x_;
        goal.target_y = target_y_;

        auto options = rclcpp_action::Client<AlignArenaTwist>::SendGoalOptions{};

        options.goal_response_callback =
            [this](const GoalHandleAlignArena::SharedPtr & handle)
            {
                if (!handle) {
                    RCLCPP_ERROR(node_->get_logger(), "[AlignArena] Goal REJECTED");
                    goal_rejected_ = true;
                } else {
                    RCLCPP_INFO(node_->get_logger(), "[AlignArena] Goal ACCEPTED");
                    goal_handle_ = handle;
                }
            };

        options.result_callback =
            [this](const GoalHandleAlignArena::WrappedResult & result)
            {
                goal_done_      = true;
                goal_succeeded_ = result.result->success;
                RCLCPP_INFO(node_->get_logger(), "[AlignArena] Result: success=%s",
                            goal_succeeded_ ? "true" : "false");
            };

        action_client_->async_send_goal(goal, options);
        goal_sent_ = true;

        RCLCPP_INFO(node_->get_logger(), "[AlignArena] Goal sent target_x=%.1f target_y=%.1f",
                    target_x_, target_y_);
        return BT::NodeStatus::RUNNING;
    }

    if (goal_rejected_)
    {
        RCLCPP_ERROR(node_->get_logger(), "[AlignArena] Goal was rejected - FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    if (goal_done_)
    {
        return goal_succeeded_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void AlignArena::onHalted()
{
    if (goal_handle_)
    {
        action_client_->async_cancel_goal(goal_handle_);
        RCLCPP_WARN(node_->get_logger(), "[AlignArena] Halted - goal cancelled.");
    }
    reset();
}

void AlignArena::reset()
{
    goal_sent_      = false;
    goal_rejected_  = false;
    goal_done_      = false;
    goal_succeeded_ = false;
    target_x_       = 0.0f;
    target_y_       = 0.0f;
    goal_handle_.reset();
}