#include "bt_nodes/tree_flow_nodes/go_wait.hpp"


GoWait::GoWait(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);


    go_wait = node_ptr_->create_subscription<std_msgs::msg::UInt8>(
    "/set_reset", qos_profile,
    std::bind(&GoWait::go_wait_subscriber_callback, this, std::placeholders::_1));
    
    go = false;
    RCLCPP_INFO(node_ptr_->get_logger(),"GoWait Initialized");
  
}


BT::NodeStatus GoWait::tick()
{
    if(go)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
    
}

void GoWait::go_wait_subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    if(msg.data == 1 )
    {
        go = true; 

    }
    else 
    {
        go = false;    
       
    }
}

