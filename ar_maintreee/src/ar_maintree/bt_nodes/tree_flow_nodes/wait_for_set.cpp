#include "bt_nodes/tree_flow_nodes/wait_for_set.hpp"

namespace
{
// Placeholder -- swap in the real trust hex value once known.
constexpr uint8_t TRUST_TRIGGER_VALUE = 0x01;

// byte value -> ROI classification string, per spec (0,1,2,3 -> real/fake/mr/none)
std::string byte_to_roi_class(uint8_t b)
{
  switch (b)
  {
    case 0: return "none";
    case 1: return "real";
    case 2: return "mr";
    case 3: return "fake";
    default: return "none";  // unexpected value, fall back safe
  }
}
}  // namespace

WaitForSet::WaitForSet(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    auto cb_group = node_ptr_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = cb_group;

    set_reset = node_ptr_->create_subscription<std_msgs::msg::UInt8>("set_reset", qos_profile, std::bind(&WaitForSet::set_reset_subscriber_callback,this,std::placeholders::_1), options);
    trust_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>("trust_data", qos_profile, std::bind(&WaitForSet::trust_callback,this,std::placeholders::_1), options);
    forest_blocks_display = node_ptr_->create_subscription<std_msgs::msg::UInt8MultiArray>("forest_data", qos_profile, std::bind(&WaitForSet::forest_blocks_callback,this,std::placeholders::_1), options);

    pick_dropped_sub = node_ptr_->create_subscription<std_msgs::msg::UInt8>("pick_dropped_data", qos_profile, std::bind(&WaitForSet::pick_dropped_callback,this,std::placeholders::_1), options);
    second_row_sub = node_ptr_->create_subscription<std_msgs::msg::UInt8>("second_row_data", qos_profile, std::bind(&WaitForSet::second_row_callback,this,std::placeholders::_1), options);
    third_row_sub = node_ptr_->create_subscription<std_msgs::msg::UInt8>("third_row_data", qos_profile, std::bind(&WaitForSet::third_row_callback,this,std::placeholders::_1), options);

    RCLCPP_INFO(node_ptr_->get_logger(),"WaitForSet Initialized");
    wait = true;
}

BT::PortsList WaitForSet::providedPorts()
{
    return {
    BT::OutputPort<MartialStageDetections>("detections_stg_1"),
    BT::OutputPort<MartialStageDetections>("detections_stg_2"),
    BT::OutputPort<MartialStageDetections>("detections_stg_3"),
    BT::OutputPort<MartialStageDetections>("detections_stg_4"),
    BT::OutputPort<uint8_t>("pick_dropped"),
    BT::OutputPort<uint8_t>("second_row"),
    BT::OutputPort<std::string>("row"),
    BT::OutputPort<uint8_t>("third_row"),
    BT::InputPort<std::string>("color"),
    BT::OutputPort<uint8_t>("trust"),
    };
}

BT::NodeStatus WaitForSet::tick()
{
    auto blackboard = config().blackboard;
    auto color_ = getInput<std::string>("color").value_or("red");

    (void)blackboard->get("sick_tictac_middle", SICK_TICTAC_MIDDLE);
    (void)blackboard->get("sick_tictac_left", SICK_TICTAC_LEFT);
    (void)blackboard->get("sick_tictac_right", SICK_TICTAC_RIGHT);

    (void)blackboard->get("in_martial", in_martial);
    (void)blackboard->get("spear_assembled", spear_assembled);
    (void)blackboard->get("scroll_picked_from_entrance", scroll_picked_from_entrance);
    (void)blackboard->get("climb_position_reached", climb_position_reached);
    (void)blackboard->get("pick_from_entrance", pick_from_entrance);
    (void)blackboard->get("store", store);
    (void)blackboard->get("carry", carry);

    (void)blackboard->get("climbed_three_photos", climbed_three_photos);

    (void)blackboard->get("in_forest", in_forest);
    (void)blackboard->get("photo_sequence", photo_sequence);

    (void)blackboard->get("in_arena", in_arena);
    (void)blackboard->get("nav_to_rack", nav_to_rack);
    (void)blackboard->get("placed_1", placed_1);
    (void)blackboard->get("placed_2", placed_2);
    (void)blackboard->get("color", color_);

    setOutput("pick_dropped", pick_dropped_);
    setOutput("second_row", second_row_);
    setOutput("third_row", third_row_);
    setOutput("trust", trust_value_);
    setOutput("row", row);

    // RCLCPP_INFO(node_ptr_->get_logger(), "WaitForSet: in_arena=%d, pick_dropped=%d, second_row=%d, third_row=%d, color=%s", in_arena, pick_dropped_, second_row_, third_row_, color_.c_str());
   
    if(!wait && start) //This node is always ticked, hence it is suitable to be the one assigning flags the MOMENT they are set
    {   
        if(in_arena==1){ 
            flag_alter(config().blackboard, FLAG_ALTER_CASE::ARENA_ENTERED);
            return BT::NodeStatus::SUCCESS;
        }

        else if(in_martial==1 && spear_assembled == 1 && scroll_picked_from_entrance==1 && climbed_three_photos==1){
            flag_alter(config().blackboard, FLAG_ALTER_CASE::CLIMBED_THREE_PHOTOS);
            return BT::NodeStatus::SUCCESS;
        }

        else if(in_martial==1 && spear_assembled == 1 && scroll_picked_from_entrance==1){
            flag_alter(config().blackboard, FLAG_ALTER_CASE::SCROLL_PICKED_FROM_ENTRANCE);
            return BT::NodeStatus::SUCCESS;
        }

        else if(in_martial==1 && spear_assembled==1){
            flag_alter(config().blackboard, FLAG_ALTER_CASE::SPEAR_ASSEMBLED);
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::SUCCESS;
    }

    if(start == false){
        if(in_arena == 1){ //When button is pressed and robot is in arena, check for pick_dropped and second_row or third_row values to set the tictac_place_setpoint
            if(pick_dropped_ == 1){
                if(third_row_ != 0){
                    row = "3";
                    if(color_ == "red"){
                        switch(third_row_){
                            case 1: blackboard->set("tictac_place_setpoint", "SICK_TICTAC_RIGHT"); break;
                            case 2: blackboard->set("tictac_place_setpoint", "SICK_TICTAC_MIDDLE"); break;
                            case 3: blackboard->set("tictac_place_setpoint", "SICK_TICTAC_LEFT"); break;
                            default: RCLCPP_ERROR(node_ptr_->get_logger(), "WaitForSet: Invalid third_row value: %d", third_row_);
                        }
                    }
                    else if(color_ == "blue"){
                        switch(third_row_){
                            case 1: blackboard->set("tictac_place_setpoint", "SICK_TICTAC_LEFT"); break;
                            case 2: blackboard->set("tictac_place_setpoint", "SICK_TICTAC_MIDDLE"); break;
                            case 3: blackboard->set("tictac_place_setpoint", "SICK_TICTAC_RIGHT"); break;
                            default: RCLCPP_ERROR(node_ptr_->get_logger(), "WaitForSet: Invalid third_row value: %d", third_row_);
                        }
                    }
                }
                else if(second_row_ != 0){
                    row = "2";
                    if(color_ == "red"){
                        switch(second_row_){
                            case 1: blackboard->set("tictac_place_setpoint", "SICK_TICTAC_RIGHT"); break;
                            case 2: blackboard->set("tictac_place_setpoint", "SICK_TICTAC_MIDDLE"); break;
                            case 3: blackboard->set("tictac_place_setpoint", "SICK_TICTAC_LEFT"); break;
                            default: RCLCPP_ERROR(node_ptr_->get_logger(), "WaitForSet: Invalid second_row value: %d", second_row_);
                        }
                    }
                    else if(color_ == "blue"){
                        switch(second_row_){
                            case 1: blackboard->set("tictac_place_setpoint", "SICK_TICTAC_LEFT"); break;
                            case 2: blackboard->set("tictac_place_setpoint", "SICK_TICTAC_MIDDLE"); break;
                            case 3: blackboard->set("tictac_place_setpoint", "SICK_TICTAC_RIGHT"); break;
                            default: RCLCPP_ERROR(node_ptr_->get_logger(), "WaitForSet: Invalid second_row value: %d", second_row_);
                        }
                    }
                }
                else{
                    RCLCPP_ERROR(node_ptr_->get_logger(), "WaitForSet: pick_dropped is set but no second_row or third_row value received");
                }
            }
        }

        if (forest_blocks_received_) //When button is pressed and robot is not in arena, send the forest_blocks data to the blackboard
        {
            setOutput("detections_stg_1", stage1_);
            setOutput("detections_stg_2", stage2_);
            setOutput("detections_stg_3", stage3_);
            setOutput("detections_stg_4", stage4_);
        }
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void WaitForSet::set_reset_subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    if(msg.data == 1 && wait )
    {
        start = true;    
        wait = false;

    }
    else if (msg.data == 0)
    {
        start = false;    
        wait = true;
    }
}

void WaitForSet::forest_blocks_callback(std_msgs::msg::UInt8MultiArray msg)
{
    if (msg.data.size() < 12)
    {
        RCLCPP_WARN(node_ptr_->get_logger(),
            "forest_blocks: expected 12 bytes, got %zu, ignoring", msg.data.size());
        return;
    }

    // bytes 0-2 -> stage1 roi1-3, bytes 3-5 -> stage2 roi1-3,
    // bytes 6-8 -> stage3 roi1-3, bytes 9-11 -> stage4 roi1-3
    stage1_.roi1 = byte_to_roi_class(msg.data[0]);
    stage1_.roi2 = byte_to_roi_class(msg.data[1]);
    stage1_.roi3 = byte_to_roi_class(msg.data[2]);

    stage2_.roi1 = byte_to_roi_class(msg.data[3]);
    stage2_.roi2 = byte_to_roi_class(msg.data[4]);
    stage2_.roi3 = byte_to_roi_class(msg.data[5]);

    stage3_.roi1 = byte_to_roi_class(msg.data[6]);
    stage3_.roi2 = byte_to_roi_class(msg.data[7]);
    stage3_.roi3 = byte_to_roi_class(msg.data[8]);

    stage4_.roi1 = byte_to_roi_class(msg.data[9]);
    stage4_.roi2 = byte_to_roi_class(msg.data[10]);
    stage4_.roi3 = byte_to_roi_class(msg.data[11]);

    forest_blocks_received_ = true;

    if(start == false){
        RCLCPP_INFO(node_ptr_->get_logger(),
        "[ForestBlocks] s1:[%s,%s,%s] s2:[%s,%s,%s] s3:[%s,%s,%s] s4:[%s,%s,%s]",
        stage1_.roi1.c_str(), stage1_.roi2.c_str(), stage1_.roi3.c_str(),
        stage2_.roi1.c_str(), stage2_.roi2.c_str(), stage2_.roi3.c_str(),
        stage3_.roi1.c_str(), stage3_.roi2.c_str(), stage3_.roi3.c_str(),
        stage4_.roi1.c_str(), stage4_.roi2.c_str(), stage4_.roi3.c_str());
    }
}

void WaitForSet::trust_callback(std_msgs::msg::UInt8 msg)
{
    trust_value_ = msg.data;
}

void WaitForSet::pick_dropped_callback(std_msgs::msg::UInt8 msg)
{
    pick_dropped_ = msg.data;
}

void WaitForSet::second_row_callback(std_msgs::msg::UInt8 msg)
{
    second_row_ = msg.data;
}

void WaitForSet::third_row_callback(std_msgs::msg::UInt8 msg)
{
    third_row_ = msg.data;
}