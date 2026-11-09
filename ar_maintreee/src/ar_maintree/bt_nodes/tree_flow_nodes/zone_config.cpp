#include "bt_nodes/tree_flow_nodes/zone_config.hpp"


ZoneConfig::ZoneConfig(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);


    zone_data = node_ptr_->create_subscription<std_msgs::msg::UInt8>(
    "zone_data", qos_profile,
    std::bind(&ZoneConfig::zone_data_subscriber_callback, this, std::placeholders::_1));
    
    zone = "0";
    RCLCPP_INFO(node_ptr_->get_logger(),"ZoneConfig Initialized");
  
}

BT::PortsList ZoneConfig::providedPorts()
{
    return {
        BT::OutputPort<std::string>("zone"),
    };
}

BT::NodeStatus ZoneConfig::tick()
{
    setOutput("zone", zone);
    return BT::NodeStatus::SUCCESS;
    
}

void ZoneConfig::zone_data_subscriber_callback(std_msgs::msg::UInt8 msg)
{  
    // zone = msg.data;
    if(msg.data == 0)
        zone = "0";
    else if(msg.data == 1)
        zone = "1";
    else
        zone = "2";
}

