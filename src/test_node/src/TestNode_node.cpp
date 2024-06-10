#include "test_node/TestNode_node.hpp"

// For _1
using namespace std::placeholders;

TestNode::TestNode(const rclcpp::NodeOptions& options) : Node("ObjPlannerNode", options) {}
