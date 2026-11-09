
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"




using namespace std::placeholders;

class CheckForFallenScroll: public BT::SyncActionNode
{
    public:
    CheckForFallenScroll(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node);
    
        rclcpp::Node::SharedPtr node_;
        
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;
    
    

};
