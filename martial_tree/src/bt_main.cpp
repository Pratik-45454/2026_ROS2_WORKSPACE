#include "bt_main.hpp"
#include "go_to_spear.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <chrono>

using namespace std::chrono_literals;

tree_ticker::tree_ticker(const std::string &node_name)
: rclcpp::Node(node_name)
{
    RCLCPP_INFO(get_logger(), "tree_ticker constructed");
}

void tree_ticker::setup()
{
    create_behavior_tree();
    timer_ = create_wall_timer(50ms, std::bind(&tree_ticker::update_behavior_tree, this));
}

void tree_ticker::create_behavior_tree()
{
    register_custom_action_nodes();

    const std::string bt_xml =
        ament_index_cpp::get_package_share_directory("martial_tree") +
        "/bt_xml/bt.xml";

    RCLCPP_INFO(get_logger(), "Looking for BT XML at: %s", bt_xml.c_str());
    tree_ = factory.createTreeFromFile(bt_xml);

    std::ofstream out("/tmp/bt_node_models.xml");
    out << BT::writeTreeNodesModelXML(factory);
    out.close();
}

void tree_ticker::register_custom_action_nodes()
{
    factory.registerBuilder<GoToSpear>(
        "GoToSpear",
        [self = shared_from_this()](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<GoToSpear>(name, config, self);
        }
    );

    factory.registerBuilder<GoToForest>(
        "GoToForest",
        [self = shared_from_this()](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<GoToForest>(name, config, self);
        }
    );
}

void tree_ticker::update_behavior_tree()
{
    auto status = tree_.tickRoot();
    if (status != BT::NodeStatus::RUNNING)
    {
        RCLCPP_INFO(get_logger(), "Behavior Tree finished with status %d", static_cast<int>(status));
        timer_->cancel();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tree_ticker>("tree_ticker");
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
