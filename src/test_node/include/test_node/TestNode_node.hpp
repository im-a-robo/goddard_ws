#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class TestNode : public rclcpp::Node {
private:
    void test_cb();

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr test_pub;
    std::shared_ptr<rclcpp::WallTimer<std::_Bind<void (TestNode::*(TestNode*))()>>> timer;

public:
    TestNode(const rclcpp::NodeOptions& options);
};