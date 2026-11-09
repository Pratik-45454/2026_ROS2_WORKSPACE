

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "action_pkg/action/move_twist.hpp"


using MoveTwist      = action_pkg::action::MoveTwist;
using GoalHandleMove = rclcpp_action::ClientGoalHandle<MoveTwist>;


class Move : public BT::StatefulActionNode
{
public:
    Move(const std::string & name,
               const BT::NodeConfig & config,
               rclcpp::Node::SharedPtr node);

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

    static BT::PortsList providedPorts();


    rclcpp::Node::SharedPtr                     node_;
    rclcpp_action::Client<MoveTwist>::SharedPtr action_client_;
    GoalHandleMove::SharedPtr                   goal_handle_;

   
    bool goal_accepted_{false};
    bool goal_rejected_{false};
    bool goal_done_{false};
    bool goal_succeeded_{false};

    float final_avg_dist_{0.0f};

    void reset();
    
    bool goal_sent_{false};
    float move_x_mm_{0.0f}, move_y_mm_{0.0f};
    float x_kp_{0.005f}, x_ki_{0.0f}, x_kd_{0.001f};
    float y_kp_{0.002f}, y_ki_{0.0f}, y_kd_{0.001f};
};

