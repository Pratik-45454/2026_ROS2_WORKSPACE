#include "bt_nodes/decision_nodes/decision_gripper_height.hpp"

DecisionGripperHeight::DecisionGripperHeight(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::SyncActionNode(name, config)
, node_(node)
{
}

BT::PortsList DecisionGripperHeight::providedPorts()
{
  return {
    BT::InputPort<float>("height"), //There was a mismatch between xml and cpp file, i couldnt find the xml so added this here
    BT::InputPort<int>("store_place"),
    BT::InputPort<int>("count"),
    BT::OutputPort<float>("elev_vert_h", "height to reach"),
  };
}

BT::NodeStatus DecisionGripperHeight::tick()
{
  auto store_place = getInput<int>("store_place").value();
  auto count = getInput<int>("count").value();

  float elev_vert_h = 0.0;

  if (store_place == 0)
  {
    if (count == 0)
      elev_vert_h = 33.0;
    else if (count == 1)
      elev_vert_h = 68.0;
  }
  else
  {
    if (count == 1)
      elev_vert_h = 33.0;
    else if (count == 2)
      elev_vert_h = 68.0;
    else
      elev_vert_h = 0.0;
  }

  setOutput("elev_vert_h", elev_vert_h);

  return BT::NodeStatus::SUCCESS;
}