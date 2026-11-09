#include "bt_nodes/tree_flow_nodes/zone_change.hpp"
#include <chrono>

using namespace std::placeholders; 
using namespace std::chrono_literals;

ZoneChange::ZoneChange(const string &name, const BT::NodeConfig &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
    {
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        set_reset_ = node_->create_subscription<std_msgs::msg::UInt8>("set_reset", 
            qos_profile, 
            std::bind(&ZoneChange::set_reset_subscriber_callback, this, std::placeholders::_1));

        emergency_stop_client_ = node_->create_client<std_srvs::srv::SetBool>("emergency_stop");
        emergency_stop_request_sent_ = false;
        emergency_stop_done_ = false;

        RCLCPP_INFO(node_->get_logger(),"ZoneChange initialized");
    }


BT::PortsList ZoneChange::providedPorts(){
    return{
    };
}

void ZoneChange::set_reset_subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg){
    set_reset_val_ = msg->data;
    current = set_reset_val_;
    if(prev - current == 1){
        emergency_stop_done_ = false;
    }

    else{
        emergency_stop_done_ = true;
    }

    prev = current;
    // RCLCPP_INFO(node_->get_logger(), "Button: %d", msg->data);
}

void ZoneChange::reset_to_martial_callback(const std_msgs::msg::UInt8::SharedPtr msg){
    reset_to_martial_ = msg->data;
}

BT::NodeStatus ZoneChange::onStart(){
    emergency_stop_done_ = false;
    return onRunning(); 
}

BT::NodeStatus ZoneChange::onRunning(){

    auto blackboard = config().blackboard;

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

    // Contradictory zone flags
    if((in_martial + in_forest + in_arena) > 1){
        RCLCPP_ERROR(node_->get_logger(), "ZoneChange: multiple zone flags set simultaneously");
        state_ = "failure";
        return BT::NodeStatus::FAILURE;
    }

    // Scroll picked but spear not assembled
    if(scroll_picked_from_entrance == 1 && spear_assembled == 0){
        RCLCPP_ERROR(node_->get_logger(), "ZoneChange: scroll picked but spear not assembled");
        state_ = "failure";
        return BT::NodeStatus::FAILURE;
    }

    // Climbed three photos but scroll not picked
    if(climbed_three_photos == 1 && scroll_picked_from_entrance == 0){
        RCLCPP_ERROR(node_->get_logger(), "ZoneChange: climbed photos but scroll not picked");
        state_ = "failure";
        return BT::NodeStatus::FAILURE;
    }

    // In arena but spear not assembled
    if(in_arena == 1 && spear_assembled == 0){
        RCLCPP_ERROR(node_->get_logger(), "ZoneChange: in arena but spear not assembled");
        state_ = "failure";
        return BT::NodeStatus::FAILURE;
    }

    if(set_reset_val_ == 0){
    
        if(in_martial == 1){
            if(last_applied_zone_ != ZoneState::MARTIAL){
                flag_alter(config().blackboard, FLAG_ALTER_CASE::GAME_START);
            
                if(spear_assembled == 1){
                    blackboard->set<std::string>("turn", "Front");
                } else {
                    if(color_ == "red")  blackboard->set<std::string>("turn", "Right");
                    else if(color_ == "blue") blackboard->set<std::string>("turn", "Left");
                }
            
                last_applied_zone_ = ZoneState::MARTIAL;
            }
            state_ = "success";
            return BT::NodeStatus::SUCCESS;
        }
    
        else if(in_forest == 1){
            if(last_applied_zone_ != ZoneState::FOREST){
                flag_alter(config().blackboard, FLAG_ALTER_CASE::RESET_IN_FOREST);
            
                if(color_ == "red")  blackboard->set<std::string>("turn", "Right");
                else if(color_ == "blue") blackboard->set<std::string>("turn", "Left");
            
                last_applied_zone_ = ZoneState::FOREST;
            }
            state_ = "success";
            return BT::NodeStatus::SUCCESS;
        }
    
        else if(in_arena == 1){
            if(last_applied_zone_ != ZoneState::ARENA){
                flag_alter(config().blackboard, FLAG_ALTER_CASE::RESET_IN_ARENA);
                last_applied_zone_ = ZoneState::ARENA;
            }
            state_ = "success";
            return BT::NodeStatus::SUCCESS;
        }
    }

    if(set_reset_val_ == 1){

        // No zone flag set when SET is pressed
        // if(in_martial == 0 && in_forest == 0 && in_arena == 0){ 
        //     RCLCPP_ERROR(node_->get_logger(), "ZoneChange: SET pressed but no zone flag set");
        //     state_ = "failure";
        //     return BT::NodeStatus::FAILURE;
        // }

        // else if(in_arena==1){ // After a node in "then" of "if then else" returns success, the if is ran again, 
        //     //this is a great way to transition the flags, done in opp order because of priority of zones
        //     flag_alter(config().blackboard, FLAG_ALTER_CASE::ARENA_ENTERED);
        //     state_ = "success";
        //     return BT::NodeStatus::SUCCESS;
        // }

        // else if(in_martial==1 && spear_assembled == 1 && scroll_picked_from_entrance==1 && climbed_three_photos==1){
        //     flag_alter(config().blackboard, FLAG_ALTER_CASE::CLIMBED_THREE_PHOTOS);
        //     state_ = "success";
        //     return BT::NodeStatus::SUCCESS;
        // }

        // else if(in_martial==1 && spear_assembled == 1 && scroll_picked_from_entrance==1){
        //     flag_alter(config().blackboard, FLAG_ALTER_CASE::SCROLL_PICKED_FROM_ENTRANCE);
        //     state_ = "success";
        //     return BT::NodeStatus::SUCCESS;
        // }

        // else if(in_martial==1 && spear_gripped==1){ 
        //     flag_alter(config().blackboard, FLAG_ALTER_CASE::SPEAR_ASSEMBLED);
        //     state_ = "success";
        //     return BT::NodeStatus::SUCCESS;
        // }

        if (!emergency_stop_done_) {
            if (!emergency_stop_request_sent_) {
                if (!emergency_stop_client_->wait_for_service(0s)) {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                        "ZoneChange: waiting for emergency_stop service...");
                    return BT::NodeStatus::RUNNING;
                }

                auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
                request->data = false;

                emergency_stop_future_ = emergency_stop_client_->async_send_request(
                    request,
                    [](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture) {}
                ).future;

                emergency_stop_request_sent_ = true;
                return BT::NodeStatus::RUNNING;
            }

            if (emergency_stop_future_.wait_for(0s) != std::future_status::ready) {
                return BT::NodeStatus::RUNNING;
            }

            auto response = emergency_stop_future_.get();

            if (!response->success) {
                RCLCPP_ERROR(node_->get_logger(),
                    "ZoneChange: emergency_stop failed: %s", response->message.c_str());
                state_ = "failure";
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(node_->get_logger(),
                "ZoneChange: emergency_stop succeeded: %s", response->message.c_str());
            emergency_stop_done_ = true;
        
        }

        if(state_ == "success") return BT::NodeStatus::SUCCESS;
        if(state_ == "failure") return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING; 
}

void ZoneChange::onHalted(){
    RCLCPP_INFO(node_->get_logger(), "ZoneChange halted");
}