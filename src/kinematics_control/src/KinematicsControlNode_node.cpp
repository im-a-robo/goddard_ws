#include "kinematics_control/KinematicsControlNode_node.hpp"

// For _1
using namespace std::placeholders;
using namespace std::chrono_literals;

KinematicsControlNode::KinematicsControlNode(const rclcpp::NodeOptions& options) : Node("KinematicsControl", options) {
    test_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 1);

    timer = this->create_wall_timer(500ms, std::bind(&KinematicsControlNode::test_cb, this));

    fl_leg_angles = {0, 0, 0};
    interface = KinematicsInterface(0.03558, 0.075, 0.0803, &fl_leg_angles);
    interface.calc_joint_deltas(0, 0.1, (0.075 + 0.0803));
}

void KinematicsControlNode::test_cb() {
    std_msgs::msg::Float64MultiArray msg;

    msg.data = {0, 0, 0, fl_leg_angles.at(0), fl_leg_angles.at(1), fl_leg_angles.at(2), 0, 0, 0, 0, 0, 0};

    test_pub->publish(msg);
}