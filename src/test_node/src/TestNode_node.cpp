#include "test_node/TestNode_node.hpp"

// For _1
using namespace std::placeholders;
using namespace std::chrono_literals;

TestNode::TestNode(const rclcpp::NodeOptions& options) : Node("ObjPlannerNode", options) {
    test_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 1);

    timer = this->create_wall_timer(500ms, std::bind(&TestNode::test_cb, this));

    RCLCPP_INFO(this->get_logger(), "Test Node Running...");
}

void TestNode::test_cb() {
    std_msgs::msg::Float64MultiArray msg;

    msg.data = {100.0, 90.0, 90.0, 20.0, 90.0, 90.0, 100.0, 90.0, 90.0, 20.0, 90.0, 90.0};

    test_pub->publish(msg);
}
