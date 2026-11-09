#include "bt_nodes/conditions/check_place_condition.hpp"   

CheckPlaceCondition::CheckPlaceCondition(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "is_rack_clear", 10,
        std::bind(&CheckPlaceCondition::subscriber_callback,this,std::placeholders::_1));
    clear= true;
    RCLCPP_INFO(node_ptr_->get_logger(),"CheckPlaceCondition::Ready");

}

BT::NodeStatus CheckPlaceCondition::tick()
{  
    if(clear)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"CheckPlaceCondition::yes");
       
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void CheckPlaceCondition::subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    clear = false;
    if(msg.data == 1)
        clear = true;  
}
