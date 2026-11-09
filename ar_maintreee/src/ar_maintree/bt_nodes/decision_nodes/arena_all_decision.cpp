#include "bt_nodes/decision_nodes/arena_all_decision.hpp"

ArenaAllDecision::ArenaAllDecision(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<tree_ticker> /*unused*/)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList ArenaAllDecision::providedPorts()
{
    return {
        BT::InputPort<MartialStageDetections>("detections", "MartialStageDetections struct from GetROIDetections"),
        BT::OutputPort<float>("move_x", "Target X movement"),
        BT::OutputPort<float>("move_y", "Target Y movement"),
        BT::OutputPort<float>("move_x_rec", "Target X movement"),
        BT::OutputPort<float>("move_y_rec", "Target Y movement"),
        BT::OutputPort<float>("align_x", "Sick alignment X setpoint"),
        BT::OutputPort<float>("align_y", "Sick alignment Y setpoint"),
        BT::OutputPort<std::string>("dxn", "Direction: L, R, or M"),
    };
}

BT::NodeStatus ArenaAllDecision::tick()
{
    MartialStageDetections detections;
    if (!getInput("detections", detections)) {
        return BT::NodeStatus::FAILURE;
    }

    const std::string & s1_roi1 = detections.stage11_roi1;
    const std::string & s1_roi2 = detections.stage11_roi2;
    const std::string & s1_roi3 = detections.stage11_roi3;
    const std::string & s2_roi1 = detections.stage11_roi4;
    const std::string & s2_roi2 = detections.stage11_roi5;
    const std::string & s2_roi3 = detections.stage11_roi6;

    double move_x = 0.0;
    double move_y = 0.0;
    double move_x_rec = 0.0;
    double move_y_rec = 0.0;
    double align_x = 0.0;
    double align_y = 0.0;
    char dxn = 'F';

    if (s2_roi2 == "no")
    {
        move_x = 1200;
        move_y = 50;

        if (s1_roi1 == "yes" && s2_roi1 == "no")
        {
            move_x_rec = 0.0;
            move_y_rec = 540;
            align_x = 0.0;   
            align_y = 0.0;   
        }
        else if (s1_roi3 == "yes" && s2_roi3 == "no")
        {
            move_x_rec = 0.0;
            move_y_rec = -540;
            align_x = 0.0;   
            align_y = 0.0;  
        }
    }
    else
    {
        if (s1_roi1 == "yes" && s2_roi1 == "no")
        {
            move_x = 1200;
            move_y = 0.0;
            move_x_rec = 0.0;
            move_y_rec = -1080;
            align_x = 0.0;   
            align_y = 0.0;   
        }
        else if (s1_roi3 == "yes" && s2_roi3 == "no")
        {
            move_x = 1200;
            move_y = 100;
            move_x_rec = 0.0;
            move_y_rec = 1080;
            align_x = 0.0;   
            align_y = 0.0;   
        }
        else
        {
            move_x = 0.0;
            move_y = 0.0;
            align_x = 0.0;
            align_y = 0.0;
        }
    }

    setOutput("move_x", move_x);
    setOutput("move_y", move_y);
    setOutput("move_x_rec", move_x_rec);
    setOutput("move_y_rec", move_y_rec);
    setOutput("align_x", align_x);
    setOutput("align_y", align_y);
    // setOutput("dxn", dxn);

    return BT::NodeStatus::SUCCESS;
}