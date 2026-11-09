#include "bt_nodes/tree_flow_nodes/init_robot.hpp"   

using namespace std::placeholders;
/*
* bit 0: /init_climb
* bit 1: /init_roll
* bit 2: /init_elev_vert
* bit 3: /init_elev_horiz
* bit 4: /init_gripper_pitch
* bit 5: /init_gripper_catch
* bit 6: ALL_INIT_BIT  fires all 6 above
* bit 7: LAUNCH_BIT    launches ak_60_up if not running
*/


InitRobot::InitRobot(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    activate_motors = node_ptr_->create_publisher<std_msgs::msg::Byte>( "/activate_motors_data", qos_profile);
   
    team_color_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>("color_data", 
                qos_profile, 
                std::bind(&InitRobot::color_data_subscriber_callback, this, _1));
    

    RCLCPP_INFO(node_ptr_->get_logger(), "InitRobot Initialized");
}

BT::PortsList InitRobot::providedPorts()
{
    return {
        BT::OutputPort<std::string>("color"),
       
    };
}

BT::NodeStatus InitRobot::onStart()
{
    set_bit_and_publish(6);

    start_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count();
    bit_set_ = true;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus InitRobot::onRunning()
{
    long now = std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
                   .count();
    setOutput("color", color_);

    if (bit_set_ && (now - start_time_) > 100)   // 100ms high
    {
        clear_bit_and_publish(6);
        bit_set_ = false;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void InitRobot::onHalted()
{
    RCLCPP_INFO(node_ptr_->get_logger(), "InitRobot::Halted");
}

void InitRobot::color_data_subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg){
    if(msg->data == 2){
        color_ = "red";
    }

    else if(msg->data == 1){
        color_ = "blue";
    }
}
void InitRobot::set_bit_and_publish(uint8_t bit)
{
    motor_bits_ |= (1 << bit);

    std_msgs::msg::Byte msg;
    msg.data = motor_bits_;
    activate_motors->publish(msg);

    RCLCPP_INFO(node_ptr_->get_logger(),
                "Published activate_motors=0x%02X (set bit %d)", motor_bits_, bit);
}

void InitRobot::clear_bit_and_publish(uint8_t bit)
{
    motor_bits_ &= ~(1 << bit);

    std_msgs::msg::Byte msg;
    msg.data = motor_bits_;
    activate_motors->publish(msg);

    RCLCPP_INFO(node_ptr_->get_logger(),
                "Published activate_motors=0x%02X (cleared bit %d)", motor_bits_, bit);
}
