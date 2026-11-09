#include "bt_nodes/actions/robotBase/climb.hpp"

#include <chrono>
#include <mutex>

using namespace BT;
using namespace std::chrono_literals;

Climb::Climb(const std::string & name,
             const BT::NodeConfig & config,
             rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config)
, node_(node)
{
    ak_up_client_   = node_->create_client<std_srvs::srv::SetBool>("ak_motor_up_execute");
    ak_down_client_ = node_->create_client<std_srvs::srv::SetBool>("ak_motor_down_exec");
    ak_stop_client_ = node_->create_client<std_srvs::srv::SetBool>("ak_motor_stop_execute");
    roll_up_client_ = node_->create_client<motor_interfaces::srv::MotorState>("ak_motor_roll_up");
}

BT::PortsList Climb::providedPorts()
{
    return {
        BT::InputPort<int>("up_down"),
        BT::InputPort<int>("next_climb_state", "Used for preset roll after retract"),
        BT::InputPort<int>("climb_state",1, "Current climb direction: UP or DOWN")
    };
}

BT::NodeStatus Climb::onStart()
{
    auto climb = getInput<int>("up_down");
    if (!climb)
    {
        RCLCPP_ERROR(node_->get_logger(), "[Climb] Missing required port 'up_down'");
        return BT::NodeStatus::FAILURE;
    }

    climb_state_ = climb.value();

    {
        std::lock_guard<std::mutex> lock(result_);
        response_received_ = false;
        call_succeeded_    = false;
    }

    goal_sent_ = false;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Climb::onRunning()
{
    if (!goal_sent_)
    {
        auto client = selectClient(climb_state_);
        if (!client)
        {
            RCLCPP_ERROR(node_->get_logger(), "[Climb] Unknown climb state: %d", climb_state_);
            return BT::NodeStatus::FAILURE;
        }

        if (!client->wait_for_service(0s))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "[Climb] Waiting for service, state: %d", climb_state_);
            return BT::NodeStatus::RUNNING;
        }

        auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        RCLCPP_INFO(node_->get_logger(), "[Climb] Sending command: %d", climb_state_);

        client->async_send_request(
            request,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
            {
                auto response = future.get();
                if (response->success)
                    RCLCPP_INFO(node_->get_logger(), "[Climb] Succeeded: %d", climb_state_);
                else
                    RCLCPP_WARN(node_->get_logger(), "[Climb] Failed: %d", climb_state_);

                std::lock_guard<std::mutex> lock(result_);
                call_succeeded_    = response->success;
                response_received_ = true;
            });

        goal_sent_ = true;
        return BT::NodeStatus::RUNNING;
    }

    std::lock_guard<std::mutex> lock(result_);

    if (!response_received_)
        return BT::NodeStatus::RUNNING;

    if (!call_succeeded_)
        return BT::NodeStatus::FAILURE;

    RCLCPP_INFO(node_->get_logger(), "[Climb] Climb complete");
    return BT::NodeStatus::SUCCESS;
}

void Climb::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "[Climb] Halted: %d", climb_state_);

    std::lock_guard<std::mutex> lock(result_);
    response_received_ = false;
    call_succeeded_    = false;
    goal_sent_         = false;
}

void Climb::sendPreset()
{
    auto next = getInput<int>("next_climb_state");
    if (!next)
    {
        RCLCPP_WARN(node_->get_logger(), "[Climb] No next_climb_state — skipping preset");
        return;
    }

    int next_state = next.value();
    float angle;

    if (next_state == static_cast<int>(ClimbState::UP))
        angle = -6.0f;
    else if (next_state == static_cast<int>(ClimbState::DOWN))
        angle = 6.0f;
    else
    {
        RCLCPP_WARN(node_->get_logger(), "[Climb] Unknown next_climb_state: %d — skipping preset", next_state);
        return;
    }

    if (!roll_up_client_->wait_for_service(200ms))
    {
        RCLCPP_WARN(node_->get_logger(), "[Climb] Preset service 'ak_motor_roll_up' not available — skipping");
        return;
    }

    auto req   = std::make_shared<motor_interfaces::srv::MotorState::Request>();
    req->data  = true;
    req->state = angle;

    RCLCPP_INFO(node_->get_logger(), "[Climb] Sending preset: ak_motor_roll_up  angle=%.1f", angle);

    roll_up_client_->async_send_request(
        req,
        [this](rclcpp::Client<motor_interfaces::srv::MotorState>::SharedFuture future)
        {
            auto response = future.get();
            RCLCPP_INFO(node_->get_logger(),
                        "[Climb] Preset response: %s",
                        response->message.c_str());
            std::lock_guard<std::mutex> lock(result_);
            preset_succeded_ = response->success;
        });
}

rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
Climb::selectClient(int state)
{
    if (state ==  1) return ak_up_client_;
    if (state == -1) return ak_down_client_;
    // if (state ==  0) return ak_stop_client_;
    return nullptr;
}