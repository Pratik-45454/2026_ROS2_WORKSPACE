#include "bt_nodes/tree_flow_nodes/initialize_ports_actuators.hpp"   

using namespace std::placeholders;

InitializePortsActuators::InitializePortsActuators(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    cmd_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());
    team_color_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>("color_data", 
                qos_profile, 
                std::bind(&InitializePortsActuators::color_data_subscriber_callback, this, _1));
    

    RCLCPP_INFO(node_ptr_->get_logger(), "InitializePortsActuators Initialized");
}

BT::PortsList InitializePortsActuators::providedPorts()
{
    return {
        BT::OutputPort<std::string>("color"),
        BT::OutputPort<std::string>("turn"),
    };
}

BT::NodeStatus InitializePortsActuators::onStart()
{
    return onRunning();
}

BT::NodeStatus InitializePortsActuators::onRunning()
{

    if(color_.empty() || (color_ != "red" && color_ != "blue")){
        return BT::NodeStatus::RUNNING;
    }

    setOutput("color", color_);
   
    if(color_ == "red") setOutput<std::string>("turn", "Right"); //Setting the yaw value for the initial turn after camera scan
    else if(color_ == "blue") setOutput<std::string>("turn", "Left");

    flag_alter(config().blackboard, FLAG_ALTER_CASE::GAME_START);
    // flag_alter(config().blackboard, FLAG_ALTER_CASE::SPEAR_ASSEMBLED); //🛑Change only this to test individual subtrees using this tree

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0f;
    cmd.linear.y = 0.0f;
    cmd.angular.z = 0.0f;
    cmd_pub_->publish(cmd);

    config().blackboard->set<std::string>("color", "red");
    config().blackboard->set<std::string>("turn", "Right");

    return BT::NodeStatus::SUCCESS;
}

void InitializePortsActuators::onHalted()
{
    RCLCPP_INFO(node_ptr_->get_logger(), "InitializePortsActuators::Halted");
}

void InitializePortsActuators::color_data_subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg){
    if(msg->data == 2){
        color_ = "red";
    }

    else if(msg->data == 1){
        color_ = "blue";
    }
}

