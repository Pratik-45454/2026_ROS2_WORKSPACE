#include "bt_nodes/decision_nodes/plan_path.hpp"
#include "bt_nodes/camera_nodes/get_roi_detections.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;

PlanPath::PlanPath(
    const std::string & name,
    const BT::NodeConfig & config,
    const rclcpp::Node::SharedPtr & node)
: BT::SyncActionNode(name, config),
  node_(node)
{
    base_graph_= {
        {2,  {{5,1},{1,1},{3,1}}},
        {1,  {{4,1},{2,1}}},
        {3,  {{6,1},{2,1}}},
        {4,  {{5,1},{7,1}}},
        {6,  {{5,1},{9,1}}},
        {9,  {{12,1},{8,1}}},
        {8,  {{11,1},{9,1},{7,1}}},
        {11, {{12,1},{10,1}}},
        {7,  {{10,1},{8,1}}},
        {5,  {{6,1},{4,1},{8,1}}},
        {10, {{11,1}}},
        {12, {{11,1}}}
    };
    
    start_node_ = 2;
   
    goal_nodes_ = {10, 12};
}

BT::PortsList PlanPath::providedPorts()
{
    return {
        BT::InputPort<MartialStageDetections>("stage1_detections"),
        BT::InputPort<MartialStageDetections>("stage2_detections"),
        BT::InputPort<MartialStageDetections>("stage3_detections"),
        BT::InputPort<MartialStageDetections>("stage4_detections"),
        BT::OutputPort<std::vector<int>>("path"),
        BT::OutputPort<std::vector<int>>("ar_nodes"),
        BT::OutputPort<int>("goal_node") 
        
    };
}
BT::NodeStatus PlanPath::tick()
{
    auto s2 =  getInput<MartialStageDetections>("stage2_detections");
    auto s3 =  getInput<MartialStageDetections>("stage3_detections");
    auto s1 =  getInput<MartialStageDetections>("stage1_detections");
    auto s4 =  getInput<MartialStageDetections>("stage4_detections");

    StageDetections stage1, stage2, stage3, stage4;
                        
    // data from camera node
    if(!s1 || !s2 || !s3 || !s4)
    {

        // RCLCPP_WARN(node_->get_logger(),
        //             "Waiting for detections...");
        //return BT::NodeStatus::FAILURE;

        RCLCPP_WARN(node_->get_logger(), "No detections, using test defaults");

        // Fake test data
        stage1 = {RoiType::REAL, RoiType::NONE, RoiType::NONE};
        stage2 = {RoiType::NONE,   RoiType::REAL, RoiType::MR};
        stage3 = {RoiType::REAL, RoiType::FAKE, RoiType::NONE};
        stage4 = {RoiType::REAL, RoiType::MR, RoiType::MR};
    }
    else
    {
        const auto &v1 = s1.value();
        stage1.roi1 = roiFromString(v1.roi1);
        stage1.roi2 = roiFromString(v1.roi2);
        stage1.roi3 = roiFromString(v1.roi3);

        const auto &v2 = s2.value();
        stage2.roi1 = roiFromString(v2.roi1);
        stage2.roi2 = roiFromString(v2.roi2);
        stage2.roi3 = roiFromString(v2.roi3);

        const auto &v3 = s3.value();
        stage3.roi1 = roiFromString(v3.roi1);
        stage3.roi2 = roiFromString(v3.roi2);
        stage3.roi3 = roiFromString(v3.roi3);

        const auto &v4 = s4.value();
        stage4.roi1 = roiFromString(v4.roi1);
        stage4.roi2 = roiFromString(v4.roi2);
        stage4.roi3 = roiFromString(v4.roi3);

        RCLCPP_INFO(node_->get_logger(),
                    "Detections: s1:[%s,%s,%s] s2:[%s,%s,%s] s3:[%s,%s,%s] s4:[%s,%s,%s]",
                    v1.roi1.c_str(), v1.roi2.c_str(), v1.roi3.c_str(),
                    v2.roi1.c_str(), v2.roi2.c_str(), v2.roi3.c_str(),
                    v3.roi1.c_str(), v3.roi2.c_str(), v3.roi3.c_str(),
                    v4.roi1.c_str(), v4.roi2.c_str(), v4.roi3.c_str());
    }
    ar_nodes.clear();
    mr_nodes.clear();
    fake_nodes.clear();

    ar_nodes.resize(12);
    mr_nodes.resize(12);
    fake_nodes.resize(12); 

    int ar_idx = 0, mr_idx = 0, fake_idx = 0;

    node_from_stage(stage1, 1, ar_idx, mr_idx, fake_idx, ar_nodes, mr_nodes, fake_nodes); //Stage1 covers nodes from 1 to 3
    node_from_stage(stage2, 4, ar_idx, mr_idx, fake_idx, ar_nodes, mr_nodes, fake_nodes); //Stage2 covers nodes from 4 to 7 and so on for the others
    node_from_stage(stage3, 7, ar_idx, mr_idx, fake_idx, ar_nodes, mr_nodes, fake_nodes);
    node_from_stage(stage4, 10, ar_idx, mr_idx, fake_idx, ar_nodes, mr_nodes, fake_nodes);

    path_planner();

    RCLCPP_INFO(node_->get_logger(), "Ar Nodes");
    for (const auto &node : ar_nodes) {
        RCLCPP_INFO(node_->get_logger(), "ar nodes: %d", node);
    }

    setOutput<std::vector<int>>("path", path);
    setOutput<std::vector<int>>("ar_nodes", ar_nodes);
    setOutput<int>("goal_node", path.empty() ? -1 : path.back()); 

    for (const auto &node : path) {
        RCLCPP_INFO(node_->get_logger(), "PlanPath: %d", node);
    }

    printf("PlanPath:sucess\n");
    return BT::NodeStatus::SUCCESS;

}

void PlanPath::node_from_stage(const StageDetections &stage, int base_node, //Assigns 3 arrays the node numbers where those respective scrolls lie
                               int &ar_idx, int &mr_idx, int &fake_idx, std::vector<int> &ar_nodes, std::vector<int> &mr_nodes, std::vector<int> &fake_nodes)
{
    RoiType rois[3] = {stage.roi1, stage.roi2, stage.roi3};
    for (int i = 0; i<3; i++)
    {
        int node_number = base_node + i;
        switch(rois[i])
        {
            case RoiType::REAL:
                ar_nodes[ar_idx] = node_number;
                ar_idx++;
                break;

            case RoiType::MR:
                mr_nodes[mr_idx] = node_number;
                mr_idx++;
                break;

            case RoiType::FAKE:
                fake_nodes[fake_idx] = node_number;
                fake_idx++;
                break;

            case RoiType::NONE:
                break;
        }
    }
}

RoiType PlanPath::roiFromString(const std::string &s)
{
    if (s == "real") return RoiType::REAL;
    if (s == "mr")   return RoiType::MR;
    if (s == "fake") return RoiType::FAKE;
    return RoiType::NONE; // covers "NONE", empty, or unrecognized values
}

void PlanPath::applyweights() //Applies weights to the initially defined base_graph using the arrays formulated using node_from_stage()
{
    
    for (auto & [node, edges] : base_graph_) {
        for (auto & edge : edges) {
            int nbr = edge.first;

            if (check_nbr(nbr, ar_nodes))       edge.second = 1; //Checks if the node in base_graph's arrays's first element...:), is in any of the arrays defined
            else if (check_nbr(nbr, mr_nodes))  edge.second = 20;
            else if (check_nbr(nbr, fake_nodes))edge.second = 100;
            else                            edge.second = 1;
        }
    }
}

bool PlanPath::check_nbr(int nbr,const std::vector<int>& nbrs)
{
    size_t len = nbrs.size();
    for(size_t i = 0; i<len ; i++)
    {
        if(nbr == nbrs[i])
         return true;
    }

    return false;
}

void PlanPath::dijkstra(int start_node,
    std::unordered_map<int,int>& dist,
    std::unordered_map<int,int>& prev)
{
    int INF = 1000000000;
    for (auto pair : base_graph_)
    {
        int node = pair.first;

        dist[node] = INF;
        prev[node] = -1;
    }
    dist[start_node] = 0;
    std::vector<int> unvisited;

    for (auto pair : base_graph_)
    {
        unvisited.push_back(pair.first);
    }

    while(!unvisited.empty())
    {
        int current = -1;
        int best_dist = INF;
        for (size_t i = 0; i < unvisited.size(); i++)
        {
            int node = unvisited[i];
            if (dist[node]< best_dist)
            {
                best_dist = dist[node];
                current = node;
            }
        }
        if (current == -1)
            break;

        for(size_t i = 0; i< unvisited.size(); i++) //Once the shortest path for a node is defined, its removed as we dont want to iterate over it again
        {
            if (unvisited[i] == current)
            {
                unvisited.erase(unvisited.begin() + i);
                break;
            }
        }
        std::vector<std::pair<int,int>> edges = base_graph_[current];
        for (size_t i = 0; i < edges.size(); i++)
        {
            int neighbor = edges[i].first;
            int weight = edges[i].second;

            int new_dist = dist[current] + weight;

            if (new_dist < dist[neighbor])
            {
                dist[neighbor] = new_dist;
                prev[neighbor] = current;
            }
        }
    }

}
void PlanPath::path_planner()
{
    applyweights();
    std::unordered_map<int, int> dist, prev;
    dijkstra(start_node_, dist, prev);

    std::vector<int> best_path;
    int best_cost = 1000000000;

    for (int goal : goal_nodes_)
    {
        std::vector<int> candidate = buildPath(goal, prev);
        int cost = dist[goal];
        if (cost < best_cost)
        {
            best_cost = cost;
            best_path = candidate;
        }
    }
    path = best_path;
}

std::vector<int> PlanPath::buildPath(int goal_node,
    const std::unordered_map<int,int>& prev)
{
    std::vector<int> path;
    int node = goal_node;
    while(node != -1)
    {
        path.insert(path.begin(), node);
        node = prev.at(node);
    }
    

    printf("Path: ");
    for (int n : path)
    {
        printf("%d ", n);
    }
    printf("\n");
    return path;

}