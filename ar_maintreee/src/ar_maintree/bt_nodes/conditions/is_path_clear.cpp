#include "bt_nodes/conditions/is_path_clear.hpp"   

isPathClear::isPathClear(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "is_path_clear", 10,
        std::bind(&isPathClear::subscriber_callback,this,std::placeholders::_1));
    clear = true;
    RCLCPP_INFO(node_ptr_->get_logger(),"isPathClear::Ready");

}

BT::NodeStatus isPathClear::tick()
{  
    if(clear)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"isPathClear::yes");
        // clear = false;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void isPathClear::subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    clear = false;
    if(msg.data == 1)
        clear = true;  
}
