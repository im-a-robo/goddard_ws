#include "kinematics_control/KinematicsControlNode_node.hpp"

// For _1
using namespace std::placeholders;
using namespace std::chrono_literals;

KinematicsControlNode::KinematicsControlNode(const rclcpp::NodeOptions& options) : Node("KinematicsControl", options) {
    test_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 1);

    timer = this->create_wall_timer(500ms, std::bind(&KinematicsControlNode::test_cb, this));

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    fl_leg_angles = {0, 0, 0};
    interface = KinematicsInterface(0.03558, 0.075, 0.0803, &fl_leg_angles);

    test_point.pose.position.x = 0;
    test_point.pose.position.y = 0;
    test_point.pose.position.z = -0.02;
    test_point.header.frame_id = "fl_foot";

    transformed = false;
}

void KinematicsControlNode::test_cb() {
    std_msgs::msg::Float64MultiArray msg;

    if (!transformed) {
        auto t = tf_buffer->lookupTransform("fl_hip", "fl_foot", tf2::TimePointZero);

        tf2::doTransform(test_point, test_point, t);

        interface.calc_joint_deltas(test_point.pose.position.x, test_point.pose.position.y, test_point.pose.position.z);
        transformed = true;
    }

    msg.data = {fl_leg_angles.at(0), fl_leg_angles.at(1), fl_leg_angles.at(2), 0, 0, 0, 0, 0, 0, 0, 0, 0};

    test_pub->publish(msg);
}