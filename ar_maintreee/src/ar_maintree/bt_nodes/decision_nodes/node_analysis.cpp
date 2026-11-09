#include "bt_nodes/decision_nodes/node_analysis.hpp"
using namespace std::placeholders;

NodeAnalysis::NodeAnalysis(
    const std::string & name,
    const BT::NodeConfig & config,
    const rclcpp::Node::SharedPtr & node
)
: BT::SyncActionNode(name, config),
  node_(node)
{
    graph_ = {
        {2,  {5, 1, 3}},
        {1,  {4, 2}},
        {3,  {6, 2}},
        {4,  {5, 7}},
        {6,  {5, 9}},
        {9,  {12, 8}},
        {8,  {11, 9, 7}},
        {11, {12, 10}},
        {7,  {10, 8}},
        {5,  {6, 4, 8}},
        {10, {11}},
        {12, {11}}
    };
}

BT::PortsList NodeAnalysis::providedPorts()
{
    return {
        BT::InputPort<std::vector<int>>("path"),
        BT::InputPort<std::vector<int>>("ar_nodes"),
        BT::OutputPort<std::vector<int>>("merged_path"),
        BT::OutputPort<std::map<int, int>>("node_state")
    };
}

BT::NodeStatus NodeAnalysis::tick()
{
    auto ar_nodes = getInput<std::vector<int>>("ar_nodes");
    auto path     = getInput<std::vector<int>>("path");

    if (!ar_nodes || !path)
    {
        RCLCPP_WARN(node_->get_logger(), "Waiting for values...");
        return BT::NodeStatus::FAILURE;
    }

    ar_scrolls   = ar_nodes.value();
    planned_path = path.value();

    printf("ar_scrolls: ");
    for (int n : ar_scrolls) printf("%d ", n);
    printf("\n");

    scroll_path();
    merge_path();

    printf("node_state:\n");
    for (auto& [node, state] : node_state)
        printf("  node %d = %d\n", node, (int)state);

    std::map<int, int> node_state_int;
    for (const auto& [key, val] : node_state)
        node_state_int[key] = static_cast<int>(val);

    setOutput("node_state",  node_state_int);
    setOutput("merged_path", merged_path);

    printf("merged_path: ");
    for (int n : merged_path) printf("%d ", n);
    printf("\n");

    printf("nodeanalysis: success\n");
    return BT::NodeStatus::SUCCESS;
}

void NodeAnalysis::scroll_path()
{
    node_state.clear();

    // mark nodes on the path
    for (size_t i = 0; i < planned_path.size(); i++)
    {
        int node  = planned_path[i];
        bool found = false;

        for (size_t j = 0; j < ar_scrolls.size(); j++)
        {
            if (node == ar_scrolls[j])
            {
                node_state[node] = SAC;
                found = true;
                break;
            }
        }

        if (!found)
            node_state[node] = C;
    }

    // mark ar_scrolls that are NOT on the path
    for (size_t j = 0; j < ar_scrolls.size(); j++)
    {
        int node = ar_scrolls[j];
        if (node == 0) continue;

        bool found = false;
        for (size_t i = 0; i < planned_path.size(); i++)
        {
            if (node == planned_path[i])
            {
                found = true;
                break;
            }
        }

        if (!found)
            node_state[node] = S;
    }
}

void NodeAnalysis::merge_path()
{
    // count how many scrolls are already on the path
    int sac_count = 0;
    for (size_t i = 0; i < planned_path.size(); i++)
        if (node_state[planned_path[i]] == SAC) sac_count++;

    // how many extra S nodes do we still need to visit
    int need = 0;
    if      (sac_count >= 2) need = 0;
    else if (sac_count == 1) need = 1;
    else                     need = 2;

    merged_path.clear();
    int s_count = 0;

    for (size_t i = 0; i < planned_path.size(); i++)
    {
        int curr_node = planned_path[i];
        merged_path.push_back(curr_node);

        if (graph_.find(curr_node) == graph_.end()) continue;

        // loop over ALL neighbors of this node and grab S nodes until we have enough
        for (size_t k = 0; k < graph_.at(curr_node).size(); k++)
        {
            if (s_count >= need) break;

            int neighbor = graph_.at(curr_node)[k];

            if (node_state.find(neighbor) != node_state.end() &&
                node_state.at(neighbor) == S)
            {
                printf("inserting s_node=%d after curr=%d\n", neighbor, curr_node);
                merged_path.push_back(neighbor);
                node_state[neighbor] = USED;
                s_count++;
            }
        }
    }
}

int NodeAnalysis::find_nearby_S(int curr_node)
{
    if (graph_.find(curr_node) == graph_.end())
        return -1;

    for (size_t i = 0; i < graph_.at(curr_node).size(); i++)
    {
        int neighbor = graph_.at(curr_node)[i];

        if (node_state.find(neighbor) != node_state.end() &&
            node_state.at(neighbor) == S)
        {
            return neighbor;
        }
    }

    return -1;
}