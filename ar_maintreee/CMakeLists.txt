cmake_minimum_required(VERSION 3.8)
project(ar_maintreee)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_pkg REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(motor_interfaces REQUIRED)
find_package(ar_interfaces_2026 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(oakd_roi_detector_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
include
)

install(DIRECTORY 
include
DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
behavior_trees
DESTINATION share/${PROJECT_NAME}
)

add_executable(bt
src/ar_maintree/bt.cpp
src/ar_maintree/bt_nodes/actions/scrollGripper/gripperPose.cpp   
src/ar_maintree/bt_nodes/actions/scrollGripper/gripperAct.cpp
src/ar_maintree/bt_nodes/actions/scrollGripper/setHeight.cpp
src/ar_maintree/bt_nodes/actions/scrollGripper/setExtension.cpp

src/ar_maintree/bt_nodes/actions/robotBase/move.cpp
src/ar_maintree/bt_nodes/actions/robotBase/climb.cpp
src/ar_maintree/bt_nodes/actions/robotBase/rotate.cpp
src/ar_maintree/bt_nodes/actions/robotBase/roll.cpp
src/ar_maintree/bt_nodes/actions/robotBase/go_to_entrance.cpp
src/ar_maintree/bt_nodes/actions/robotBase/odom_pid_client.cpp
src/ar_maintree/bt_nodes/actions/robotBase/sick_pid_client.cpp
src/ar_maintree/bt_nodes/actions/robotBase/stall_detection.cpp
src/ar_maintree/bt_nodes/actions/robotBase/vel_publish.cpp
src/ar_maintree/bt_nodes/actions/robotBase/yaw_setpoint.cpp
src/ar_maintree/bt_nodes/actions/robotBase/align_client.cpp
src/ar_maintree/bt_nodes/actions/robotBase/ak_client.cpp
src/ar_maintree/bt_nodes/actions/robotBase/align_arena.cpp

src/ar_maintree/bt_nodes/camera_nodes/find_hidden_scroll.cpp
src/ar_maintree/bt_nodes/camera_nodes/get_roi_detections.cpp

src/ar_maintree/bt_nodes/tree_flow_nodes/initialize_ports_actuators.cpp
src/ar_maintree/bt_nodes/tree_flow_nodes/wait_for_set.cpp
src/ar_maintree/bt_nodes/tree_flow_nodes/zone_change.cpp
src/ar_maintree/bt_nodes/tree_flow_nodes/zone_config.cpp
src/ar_maintree/bt_nodes/tree_flow_nodes/setpoint_loader.cpp
src/ar_maintree/bt_nodes/tree_flow_nodes/wait_for_lift.cpp
src/ar_maintree/bt_nodes/tree_flow_nodes/start_wait.cpp
src/ar_maintree/bt_nodes/tree_flow_nodes/go_wait.cpp
src/ar_maintree/bt_nodes/tree_flow_nodes/init_robot.cpp



# src/ar_maintree/bt_nodes/decision_nodes/arena_decision_node.cpp
# src/ar_maintree/bt_nodes/decision_nodes/arena_all_decision.cpp
src/ar_maintree/bt_nodes/decision_nodes/decision_odom.cpp
src/ar_maintree/bt_nodes/decision_nodes/decision_sick.cpp
src/ar_maintree/bt_nodes/decision_nodes/path_planner.cpp  
src/ar_maintree/bt_nodes/decision_nodes/node_analysis.cpp  
src/ar_maintree/bt_nodes/decision_nodes/decision_node.cpp  
src/ar_maintree/bt_nodes/decision_nodes/decision_gripper_height.cpp  

src/ar_maintree/bt_nodes/conditions/is_inside_forest.cpp
src/ar_maintree/bt_nodes/conditions/is_path_clear.cpp      
src/ar_maintree/bt_nodes/conditions/check_for_C.cpp       
src/ar_maintree/bt_nodes/conditions/check_for_S.cpp  
src/ar_maintree/bt_nodes/conditions/check_for_prox.cpp
src/ar_maintree/bt_nodes/conditions/check_for_fallen_scroll.cpp
src/ar_maintree/bt_nodes/conditions/check_place_condition.cpp
src/ar_maintree/bt_nodes/conditions/check_goal_node.cpp
src/ar_maintree/bt_nodes/conditions/always_node.cpp
src/ar_maintree/bt_nodes/conditions/brake_test_node.cpp
src/ar_maintree/bt_nodes/conditions/incremental_scroll.cpp
src/ar_maintree/bt_nodes/conditions/check_second_row.cpp

)
target_include_directories(bt PRIVATE
include
)
# target_include_directories(bt PRIVATE
# include
# /home/messsyyyyy/arrobo_ws/install/martial_tree/include/martial_tree
# )

ament_target_dependencies(bt
rclcpp
rclcpp_action
behaviortree_cpp
action_pkg
std_srvs
std_msgs
geometry_msgs
sensor_msgs
motor_interfaces
ar_interfaces_2026
nav_msgs
yaml-cpp
oakd_roi_detector_interfaces
ament_index_cpp
)

target_link_libraries(bt yaml-cpp)

install(TARGETS
  bt
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees/
)

install(DIRECTORY yaml_files/
    DESTINATION share/${PROJECT_NAME}/yaml_files
)

ament_package()