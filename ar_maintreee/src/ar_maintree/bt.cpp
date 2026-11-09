#include "bt_nodes/bt.hpp"

using namespace std::chrono_literals;

tree_ticker::tree_ticker(const std::string &node_name)
: rclcpp::Node(node_name)
{
    RCLCPP_INFO(get_logger(), "tree_ticker constructed");
}

void tree_ticker::setup()
{
    create_behavior_tree();
    timer_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    bool debug_mode = this->declare_parameter<bool>("debug", false);
    int tick_ms = debug_mode ? 2000 : 10;

    timer_ = create_wall_timer(
        std::chrono::milliseconds(tick_ms),
        std::bind(&tree_ticker::update_behavior_tree, this),
        timer_group
    );
}

void tree_ticker::create_behavior_tree()
{
    register_custom_action_nodes();
    register_actions_nodes();
    register_decision_nodes();
    register_condition_nodes();

    auto blackboard = BT::Blackboard::create();

    blackboard->set<int>("critical_failure", 0); //Used to exit the tree in case any node in the main sequence returns FAILURE.
    
    blackboard->set<int>("in_martial", 0);
    
    blackboard->set<int>("spear_assembled", 0);
    blackboard->set<int>("spear_gripped", 0); //For when the spear is gripped but we dont want the precondition to trigger prematurely
    
    blackboard->set<int>("scroll_picked_from_entrance", 0);
    
    blackboard->set<int>("store", 0);
    
    blackboard->set<int>("carry", 0);
    
    blackboard->set<int>("climbed_three_photos", 0);
    
    blackboard->set<int>("in_forest", 0);
    
    blackboard->set<int>("photo_sequence", 0);
    
    blackboard->set<int>("forest_scrolls_picked", 0);
    
    blackboard->set<int>("in_arena", 0); //change to 0
    
    blackboard->set<int>("placed_1", 0);
    
    blackboard->set<int>("placed_2", 0);
    
    blackboard->set<int>("photo_stage_1", 0);
    
    blackboard->set<uint8_t>("pick_dropped", 0);
    
    blackboard->set<uint8_t>("trust", 0);

    // std::vector<float> arr = {175.0f, 646.0f, 0.0f, 0.0f};
    // blackboard->set<std::vector<float>>("entrance_movement_", arr);
    
    // std_msgs::msg::Float32MultiArray arr;
    // arr.data = {175.0f, 646.0f, 0.0f, 0.0f};
    // blackboard->set<std_msgs::msg::Float32MultiArray>("entrance_movement", arr);

    blackboard->set<CurrentYaw>("current_yaw", CurrentYaw::FRONT);

    const std::string ar_pkg = ament_index_cpp::get_package_share_directory("ar_maintreee");
    const std::string home = std::getenv("HOME");

    try {

        //--------------------------------------Martial club sub-trees----------------------------------------------------

        factory.registerBehaviorTreeFromFile(ar_pkg + "/behavior_trees/main_trees/martial_club/climb_three_photos.xml");
        RCLCPP_INFO(get_logger(), "climb_three_photos OK");

        factory.registerBehaviorTreeFromFile(ar_pkg + "/behavior_trees/main_trees/martial_club/sick_scroll_pick_from_entrance.xml");
        RCLCPP_INFO(get_logger(), "sick_scroll_pick_from_entrance (SickScrollPickFromEntrance) OK");

        factory.registerBehaviorTreeFromFile(ar_pkg + "/behavior_trees/main_trees/martial_club/odom_scroll_pick_from_entrance.xml");
        RCLCPP_INFO(get_logger(), "odom_scroll_pick_from_entrance OK (registered, not yet wired into any tree)");

        factory.registerBehaviorTreeFromFile(ar_pkg + "/behavior_trees/main_trees/martial_club/spear_assembly.xml");
        RCLCPP_INFO(get_logger(), "spear_assembly OK");

      

        factory.registerBehaviorTreeFromFile(ar_pkg + "/behavior_trees/main_trees/mahua_forest/forest_tree.xml"); //Forest tree now 
        // contains out tree within it for ease of use of flag logic in the main tree
        RCLCPP_INFO(get_logger(), "forest_tree OK");

        factory.registerBehaviorTreeFromFile(ar_pkg + "/behavior_trees/main_trees/mahua_forest/gripper_subtree.xml");
        RCLCPP_INFO(get_logger(), "gripper_subtree OK");

        factory.registerBehaviorTreeFromFile(ar_pkg + "/behavior_trees/main_trees/arena/out_tree.xml");
        RCLCPP_INFO(get_logger(), "out_tree OK");

        factory.registerBehaviorTreeFromFile(ar_pkg + "/behavior_trees/main_trees/arena/arena.xml"); //Too many errors, uncomment it later :)
        RCLCPP_INFO(get_logger(), "arena OK");

        factory.registerBehaviorTreeFromFile(ar_pkg + "/behavior_trees/main_trees/arena/arena_run.xml");
        RCLCPP_INFO(get_logger(), "arena run OK");

    
        // factory.registerBehaviorTreeFromFile(ar_pkg + "/behavior_trees/tmp2.xml");//🛑Change only this one to register the subtree whilst testing individual subtrees :)
        // RCLCPP_INFO(get_logger(), "tmp2 OK");

        factory.registerBehaviorTreeFromFile(ar_pkg + "/behavior_trees/main_trees/main_bt.xml");
        RCLCPP_INFO(get_logger(), "MainTree OK");

        tree_ = factory.createTree("MainTree", blackboard);

        // tree_ = factory.createTree("TMP2", blackboard); //🛑Change only this one whilst testing individual subtrees :)

        tree_ = factory.createTreeFromFile(
            ar_pkg + "/behavior_trees/main_trees/main_bt.xml",
            blackboard
        );
        
        // tree_ = factory.createTreeFromFile(
        //     ar_pkg + "/behavior_trees/tmp2.xml",
        //     blackboard
        // );

        RCLCPP_INFO(get_logger(), "tree created OK");

        logger_ = std::make_unique<BT::StdCoutLogger>(tree_); 
        groot_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, 1667);
        file_logger_ = std::make_unique<BT::FileLogger2>(tree_, home + "/arrobo_ws/src/arrobo_ws/ar_maintreee/behavior_trees/bt_trace.btlog");

    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "BT error: %s", e.what());
        throw;
    }

    std::ofstream out(home + "/arrobo_ws/src/arrobo_ws/ar_maintreee/behavior_trees/bt_node_models.xml");
    out << BT::writeTreeNodesModelXML(factory);
    out.close();
}

void tree_ticker::register_custom_action_nodes()
{
    auto shared_self = std::static_pointer_cast<tree_ticker>(this->shared_from_this());

    factory.registerBuilder<SickPIDClient>(
        "SickPIDClient",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<SickPIDClient>(name, config, shared_self); }
    );

    factory.registerBuilder<GoToEntrance>(
        "GoToEntrance",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<GoToEntrance>(name, config, shared_self); }
    );

    // factory.registerBuilder<YawPIDClient>(
    //     "YawPIDClient",
    //     [shared_self](const std::string &name, const BT::NodeConfig &config)
    //     { return std::make_unique<YawPIDClient>(name, config, shared_self); }
    // );

    factory.registerBuilder<AkClient>(
        "AkClient",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<AkClient>(name, config, shared_self); }
    );

    factory.registerBuilder<YawSetpoint>(
        "YawSetpoint",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<YawSetpoint>(name, config, shared_self); }
    );

    factory.registerBuilder<GetROIDetections>(
        "GetROIDetections",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<GetROIDetections>(name, config, shared_self); }
    );

    factory.registerBuilder<FindHiddenScroll>(
        "FindHiddenScroll",
        [](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<FindHiddenScroll>(name, config); }
    );

    factory.registerBuilder<OdomPIDClient>(
        "OdomPIDClient",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<OdomPIDClient>(name, config, shared_self); }
    );

    factory.registerBuilder<InitializePortsActuators>(
        "InitializePortsActuators",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<InitializePortsActuators>(name, config, shared_self); }
    );

    factory.registerBuilder<WaitForSet>(
        "WaitForSet",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<WaitForSet>(name, config, shared_self); }
    );

    factory.registerBuilder<ZoneChange>(
        "ZoneChange",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<ZoneChange>(name, config, shared_self); }
    );


    factory.registerBuilder<ZoneConfig>(
        "ZoneConfig",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<ZoneConfig>(name, config, shared_self); }
    );

    // factory.registerBuilder<SetDestinationFromPriority>(
    //     "SetDestinationFromPriority",
    //     [shared_self](const std::string &name, const BT::NodeConfig &config)
    //     { return std::make_unique<SetDestinationFromPriority>(name, config, shared_self); }
    // );

    // factory.registerBuilder<GetLocationForStage>(
    //     "GetLocationForStage",
    //     [shared_self](const std::string &name, const BT::NodeConfig &config)
    //     { return std::make_unique<GetLocationForStage>(name, config, shared_self); }
    // );

    // factory.registerBuilder<UpdateCurrentStage>(
    //     "UpdateCurrentStage",
    //     [shared_self](const std::string &name, const BT::NodeConfig &config)
    //     { return std::make_unique<UpdateCurrentStage>(name, config, shared_self); }
    // );

    // factory.registerBuilder<CheckItemsPicked>(
    //     "CheckItemsPicked",
    //     [shared_self](const std::string &name, const BT::NodeConfig &config)
    //     { return std::make_unique<CheckItemsPicked>(name, config, shared_self); }
    // );

    factory.registerBuilder<SetpointLoader>(
        "SetpointLoader",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<SetpointLoader>(name, config); }
    );

    factory.registerBuilder<StallDetection>(
        "StallDetection",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<StallDetection>(name, config, shared_self); }
    );

    factory.registerBuilder<DecisionSick>(
        "DecisionSick",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<DecisionSick>(name, config, shared_self); }
    );

    factory.registerBuilder<DecisionOdom>(
        "DecisionOdom",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<DecisionOdom>(name, config, shared_self); }
    );

    factory.registerBuilder<WaitForLift>(
        "WaitForLift",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<WaitForLift>(name, config, shared_self); }
    );

    factory.registerBuilder<StartWait>(
        "StartWait",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<StartWait>(name, config, shared_self); }
    );


    factory.registerBuilder<GoWait>(
        "GoWait",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<GoWait>(name, config, shared_self); }
    );

    factory.registerBuilder<InitRobot>(
        "InitRobot",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<InitRobot>(name, config, shared_self); }
    );



}


void tree_ticker::register_actions_nodes()
{
    auto shared_self = std::static_pointer_cast<tree_ticker>(this->shared_from_this());

    factory.registerBuilder<Move>(
        "Move",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<Move>(name, config, shared_self); }
    );

    factory.registerBuilder<Climb>(
        "Climb",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<Climb>(name, config, shared_self); }
    );

    factory.registerBuilder<Roll>(
        "Roll",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<Roll>(name, config, shared_self); }
    );

    factory.registerBuilder<Rotate>(
        "Rotate",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<Rotate>(name, config, shared_self); }
    );

    factory.registerBuilder<GripperPose>(
        "GripperPose",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<GripperPose>(name, config, shared_self); }
    );

    factory.registerBuilder<SetExtension>(
        "setExtension",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<SetExtension>(name, config, shared_self); }
    );

    factory.registerBuilder<SetHeight>(
        "setHeight",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<SetHeight>(name, config, shared_self); }
    );

    factory.registerBuilder<GripperAct>(
        "GripperAct",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<GripperAct>(name, config, shared_self); }
    );

    factory.registerBuilder<VelPublish>(
        "VelPublish",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<VelPublish>(name, config, shared_self); }
    );

    factory.registerBuilder<AlignArena>(
        "AlignArena",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<AlignArena>(name, config, shared_self); }
    );
}

void tree_ticker::register_decision_nodes()
{
    auto shared_self = std::static_pointer_cast<tree_ticker>(this->shared_from_this());

    factory.registerBuilder<PlanPath>(
        "PlanPath",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<PlanPath>(name, config, shared_self); }
    );

    factory.registerBuilder<DecisionNode>(
        "DecisionNode",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<DecisionNode>(name, config, shared_self); }
    );

    factory.registerBuilder<NodeAnalysis>(
        "NodeAnalysis",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<NodeAnalysis>(name, config, shared_self); }
    );

    // factory.registerBuilder<ArenaDecisionNode>(
    //     "ArenaDecisionNode",
    //     [shared_self](const std::string &name, const BT::NodeConfig &config)
    //     { return std::make_unique<ArenaDecisionNode>(name, config, shared_self); }
    // );

    // factory.registerBuilder<ArenaAllDecision>(
    //     "ArenaAllDecision",
    //     [shared_self](const std::string &name, const BT::NodeConfig &config)
    //     { return std::make_unique<ArenaAllDecision>(name, config, shared_self); }
    // );

    factory.registerBuilder<DecisionGripperHeight>(
        "DecisionGripperHeight",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<DecisionGripperHeight>(name, config, shared_self); }
    );


}

void tree_ticker::register_condition_nodes()
{
    auto shared_self = std::static_pointer_cast<tree_ticker>(this->shared_from_this());

    factory.registerBuilder<isInsideForest>(
        "IsInsideForest",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<isInsideForest>(name, config, shared_self); }
    );

    factory.registerBuilder<isPathClear>(
        "isPathClear",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<isPathClear>(name, config, shared_self); }
    );

    factory.registerBuilder<CheckForC>(
        "CheckforC",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<CheckForC>(name, config, shared_self); }
    );

    factory.registerBuilder<CheckForS>(
        "CheckforS",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<CheckForS>(name, config, shared_self); }
    );

    factory.registerBuilder<CheckForProx>(
        "CheckForProx",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<CheckForProx>(name, config, shared_self); }
    );

    factory.registerBuilder<CheckGoalNode>(
        "CheckGoalNode",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<CheckGoalNode>(name, config, shared_self); }
    );

    factory.registerBuilder<CheckPlaceCondition>(
        "CheckPlaceCondition",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<CheckPlaceCondition>(name, config, shared_self); }
    );
   

    factory.registerBuilder<CheckForFallenScroll>(
        "CheckForFallenScroll",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<CheckForFallenScroll>(name, config, shared_self); }
    );


    factory.registerBuilder<CheckSecondRow>(
        "CheckSecondRow",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<CheckSecondRow>(name, config, shared_self); }
    );

    factory.registerBuilder<AlwaysNode>(
        "AlwaysNode",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<AlwaysNode>(name, config, shared_self); }
    );

    factory.registerBuilder<BrakeTest>(
        "BrakeTest",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        { return std::make_unique<BrakeTest>(name, config, shared_self); }
    );
    factory.registerBuilder<IncrementalScroll>(
    "IncrementalScroll",
    [shared_self](const std::string& name, const BT::NodeConfig& config)
    { return std::make_unique<IncrementalScroll>(name, config, shared_self); }
);
}

void tree_ticker::update_behavior_tree()
{
    bool debug_mode = this->get_parameter("debug").as_bool();
    auto status = debug_mode ? tree_.tickOnce() : tree_.tickWhileRunning();
    if (status != BT::NodeStatus::RUNNING)
    {
        RCLCPP_INFO(
            get_logger(),
            "Behavior Tree finished with status %d",
            static_cast<int>(status)
        );
        timer_->cancel();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tree_ticker>("bt_manager_node");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    node->setup();
    executor.spin();

    rclcpp::shutdown();
    return 0;
}