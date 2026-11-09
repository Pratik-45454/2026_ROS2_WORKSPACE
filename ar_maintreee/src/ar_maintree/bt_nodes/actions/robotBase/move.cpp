#include "bt_nodes/actions/robotBase/move.hpp"

Move::Move(const std::string & name,
           const BT::NodeConfig & config,
           rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config)
, node_(node)
{
    action_client_ = rclcpp_action::create_client<MoveTwist>(node_, "move_twist_");
}

BT::PortsList Move::providedPorts()
{
    return {
        BT::InputPort<float>("move_x"),
        BT::InputPort<float>("move_y"),
        BT::InputPort<float>("x_kp", 0.005f, "X proportional gain"),
        BT::InputPort<float>("x_ki", 0.0f,   "X integral gain"),
        BT::InputPort<float>("x_kd", 0.001f, "X derivative gain"),
        BT::InputPort<float>("y_kp", 0.002f, "Y proportional gain"),
        BT::InputPort<float>("y_ki", 0.0f,   "Y integral gain"),
        BT::InputPort<float>("y_kd", 0.001f, "Y derivative gain"),
    };
}

BT::NodeStatus Move::onStart()
{
    reset();

    getInput("move_x", move_x_mm_);
    getInput("move_y", move_y_mm_);
    getInput("x_kp", x_kp_);
    getInput("x_ki", x_ki_);
    getInput("x_kd", x_kd_);
    getInput("y_kp", y_kp_);
    getInput("y_ki", y_ki_);
    getInput("y_kd", y_kd_);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Move::onRunning()
{
    if (!goal_sent_)
    {
        if (!action_client_->wait_for_action_server(std::chrono::milliseconds(0)))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "[Move] Waiting for move_twist server...");
            return BT::NodeStatus::RUNNING;
        }

        MoveTwist::Goal goal;
        goal.move_x = move_x_mm_;
        goal.move_y = move_y_mm_;
        goal.x_kp   = x_kp_;
        goal.x_ki   = x_ki_;
        goal.x_kd   = x_kd_;
        goal.y_kp   = y_kp_;
        goal.y_ki   = y_ki_;
        goal.y_kd   = y_kd_;

        auto options = rclcpp_action::Client<MoveTwist>::SendGoalOptions{};

        options.goal_response_callback =
            [this](const GoalHandleMove::SharedPtr & handle)
            {
                if (!handle) {
                    RCLCPP_ERROR(node_->get_logger(), "[Move] Goal REJECTED");
                    goal_rejected_ = true;
                } else {
                    RCLCPP_INFO(node_->get_logger(), "[Move] Goal ACCEPTED");
                    goal_handle_   = handle;
                    goal_accepted_ = true;
                }
            };

        options.feedback_callback =
            [this](GoalHandleMove::SharedPtr,
                   const std::shared_ptr<const MoveTwist::Feedback>)
            {
            };

        options.result_callback =
            [this](const GoalHandleMove::WrappedResult & result)
            {
                goal_done_      = true;
                goal_succeeded_ = result.result->success;
                final_avg_dist_ = result.result->final_avg_dist_m;
                RCLCPP_INFO(node_->get_logger(),
                            "[Move] Result: success=%s  final_dist=%.4f m",
                            goal_succeeded_ ? "true" : "false",
                            final_avg_dist_);
            };

        action_client_->async_send_goal(goal, options);
        goal_sent_ = true;

        RCLCPP_INFO(node_->get_logger(),
                    "[Move] Goal sent move_x=%.1f mm  move_y=%.1f mm",
                    move_x_mm_, move_y_mm_);
        return BT::NodeStatus::RUNNING;
    }

    if (goal_rejected_)
    {
        RCLCPP_ERROR(node_->get_logger(), "[Move] Goal was rejected - FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    if (goal_done_)
    {
        if (goal_succeeded_)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "[Move] Succeeded - avg_dist=%.4f m", final_avg_dist_);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "[Move] Server returned failure");
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::RUNNING;
}

void Move::onHalted()
{
    if (goal_handle_)
    {
        action_client_->async_cancel_goal(goal_handle_);
        RCLCPP_WARN(node_->get_logger(), "[Move] Halted - goal cancelled.");
    }
    reset();
}

void Move::reset()
{
    goal_sent_      = false;
    goal_accepted_  = false;
    goal_rejected_  = false;
    goal_done_      = false;
    goal_succeeded_ = false;
    final_avg_dist_ = 0.0f;
    move_x_mm_      = 0.0f;
    move_y_mm_      = 0.0f;
    goal_handle_.reset();
}