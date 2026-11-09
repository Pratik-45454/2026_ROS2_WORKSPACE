#ifndef BT_H
#define BT_H


#include <fstream>
#include <chrono>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>


#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>


#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>

#include "bt_nodes/actions/scrollGripper/gripperPose.hpp"
#include "bt_nodes/actions/scrollGripper/gripperAct.hpp"
#include "bt_nodes/actions/scrollGripper/setHeight.hpp"
#include "bt_nodes/actions/scrollGripper/setExtension.hpp"

#include "bt_nodes/actions/robotBase/move.hpp"
#include "bt_nodes/actions/robotBase/climb.hpp"
#include "bt_nodes/actions/robotBase/rotate.hpp"
#include "bt_nodes/actions/robotBase/roll.hpp"
#include "bt_nodes/actions/robotBase/go_to_entrance.hpp"
#include "bt_nodes/actions/robotBase/odom_pid_client.hpp"
#include "bt_nodes/actions/robotBase/sick_pid_client.hpp"
#include "bt_nodes/actions/robotBase/stall_detection.hpp"
#include "bt_nodes/actions/robotBase/vel_publish.hpp"
#include "bt_nodes/actions/robotBase/yaw_setpoint.hpp"
#include "bt_nodes/actions/robotBase/align_client.hpp"
#include "bt_nodes/actions/robotBase/ak_client.hpp"
#include "bt_nodes/actions/robotBase/align_arena.hpp"


#include "bt_nodes/camera_nodes/find_hidden_scroll.hpp"
#include "bt_nodes/camera_nodes/get_roi_detections.hpp"

#include "bt_nodes/decision_nodes/arena_decision_node.hpp"  
#include "bt_nodes/decision_nodes/arena_all_decision.hpp"  
#include "bt_nodes/decision_nodes/decision_odom.hpp"
#include "bt_nodes/decision_nodes/decision_sick.hpp"
#include "bt_nodes/decision_nodes/plan_path.hpp"         
#include "bt_nodes/decision_nodes/node_analysis.hpp"
#include "bt_nodes/decision_nodes/decision_node.hpp"
#include "bt_nodes/decision_nodes/decision_gripper_height.hpp"


#include "bt_nodes/conditions/is_inside_forest.hpp"
#include "bt_nodes/conditions/is_path_clear.hpp"
#include "bt_nodes/conditions/check_for_C.hpp"
#include "bt_nodes/conditions/check_for_S.hpp"
#include "bt_nodes/conditions/check_for_prox.hpp"
#include "bt_nodes/conditions/check_goal_node.hpp"
#include "bt_nodes/conditions/always_node.hpp"
#include "bt_nodes/conditions/brake_test_node.hpp"
#include "bt_nodes/conditions/incremental_scroll.hpp"
#include "bt_nodes/conditions/check_for_fallen_scroll.hpp"
#include "bt_nodes/conditions/check_place_condition.hpp"
#include "bt_nodes/conditions/check_second_row.hpp"



#include "bt_nodes/tree_flow_nodes/initialize_ports_actuators.hpp"
#include "bt_nodes/tree_flow_nodes/wait_for_set.hpp"
#include "bt_nodes/tree_flow_nodes/zone_change.hpp"
#include "bt_nodes/tree_flow_nodes/zone_config.hpp"
#include "bt_nodes/tree_flow_nodes/setpoint_loader.hpp"
#include "bt_nodes/tree_flow_nodes/wait_for_lift.hpp"
#include "bt_nodes/tree_flow_nodes/start_wait.hpp"
#include "bt_nodes/tree_flow_nodes/go_wait.hpp"
#include "bt_nodes/tree_flow_nodes/init_robot.hpp"




#include "bt_nodes/actions/scrollGripper/gripperAct.hpp"
#include "bt_nodes/actions/scrollGripper/gripperPose.hpp"
#include "bt_nodes/actions/scrollGripper/setExtension.hpp"
#include "bt_nodes/actions/scrollGripper/setHeight.hpp"


class tree_ticker : public rclcpp::Node
{
public:
    explicit tree_ticker(const std::string &node_name);

    void setup();
    void create_behavior_tree();
    void update_behavior_tree();

    void register_custom_action_nodes();
    void register_actions_nodes();
    void register_decision_nodes();
    void register_condition_nodes();

private:
    rclcpp::CallbackGroup::SharedPtr timer_group; 
    rclcpp::TimerBase::SharedPtr     timer_;

    BT::Tree                         tree_;
    BT::BehaviorTreeFactory          factory;       

    std::unique_ptr<BT::StdCoutLogger>   logger_;
    std::unique_ptr<BT::Groot2Publisher> groot_publisher_;
    std::unique_ptr<BT::FileLogger2>     file_logger_;
};

#endif // BT_H