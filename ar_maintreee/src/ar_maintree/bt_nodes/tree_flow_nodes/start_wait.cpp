#include "bt_nodes/tree_flow_nodes/start_wait.hpp"


StartWait::StartWait(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name, config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    rclcpp::QoS init_robot_qos_profile(10);
    init_robot_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    set_reset = node_ptr_->create_subscription<std_msgs::msg::UInt8>(
        "/set_reset", qos_profile,
        std::bind(&StartWait::set_reset_subscriber_callback, this, std::placeholders::_1));

    init_robot = node_ptr_->create_subscription<std_msgs::msg::UInt8>(
        "/init_robot", init_robot_qos_profile,
        std::bind(&StartWait::init_robot_subscriber_callback, this, std::placeholders::_1));

    start = false;
    RCLCPP_INFO(node_ptr_->get_logger(), "StartWait Initialized");
}


BT::NodeStatus StartWait::tick()
{
    if(start)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
    
}

void StartWait::set_reset_subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    if(msg.data == 0 && prev_set_data == 1)
    {
        start = false;  
    }
    prev_set_data = msg.data;

}

void StartWait::init_robot_subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    if(msg.data != 0 )
    {
        start = true; 
    }
}

