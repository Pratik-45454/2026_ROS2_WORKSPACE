#include "bt_nodes/conditions/check_second_row.hpp"   

CheckSecondRow::CheckSecondRow(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "is_second_row", 10,
        std::bind(&CheckSecondRow::subscriber_callback,this,std::placeholders::_1));
    second = true;
    RCLCPP_INFO(node_ptr_->get_logger(),"CheckSecondRow::Ready");

}

BT::NodeStatus CheckSecondRow::tick()
{  
    if(second)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"CheckSecondRow::yes");
    
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void CheckSecondRow::subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    second = false;
    if(msg.data == 1)
        second = true;  
}
