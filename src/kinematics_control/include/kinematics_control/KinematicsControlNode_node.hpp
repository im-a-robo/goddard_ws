#pragma once

#include "KinematicsInterface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class KinematicsControlNode : public rclcpp::Node {
private:
    void test_cb();

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr test_pub;
    std::shared_ptr<rclcpp::WallTimer<std::_Bind<void (KinematicsControlNode::*(KinematicsControlNode*))()>>> timer;

    std::vector<double> fl_leg_angles;

    KinematicsInterface interface;

public:
    KinematicsControlNode(const rclcpp::NodeOptions& options);
};