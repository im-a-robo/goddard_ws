#include "kinematics_control/KinematicsControlNode_node.hpp"

// For _1
using namespace std::placeholders;
using namespace std::chrono_literals;

KinematicsControlNode::KinematicsControlNode(const rclcpp::NodeOptions& options) : Node("KinematicsControl", options) {
    this->test_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 1);
    this->test_point_mk_pub = this->create_publisher<visualization_msgs::msg::Marker>("/test_point_vis", 1);

    timer = this->create_wall_timer(500ms, std::bind(&KinematicsControlNode::test_cb, this));

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    fl_leg_angles = {0, 0, 0};
    interface = KinematicsInterface(0.03558, 0.075, 0.0803, &fl_leg_angles);

    test_point.pose.position.x = 0.015;
    test_point.pose.position.y = 0.015;
    test_point.pose.position.z = 0.05;
    test_point.header.frame_id = "fl_foot";

    transformed = false;
}

void KinematicsControlNode::test_cb() {
    std_msgs::msg::Float64MultiArray msg;

    if (!transformed) {
        auto t1 = tf_buffer->lookupTransform("fl_hip", "fl_foot", tf2::TimePointZero);

        tf2::doTransform(test_point.pose, test_point.pose, t1);

        interface.calc_joint_deltas(test_point.pose.position.x, test_point.pose.position.y, test_point.pose.position.z);

        auto t2 = tf_buffer->lookupTransform("base_link", "fl_hip", tf2::TimePointZero);

        tf2::doTransform(test_point.pose, test_point.pose, t2);

        test_point_mk.header.frame_id = "base_link";
        test_point_mk.header.stamp = get_clock()->now();
        test_point_mk.action = visualization_msgs::msg::Marker::ADD;
        test_point_mk.type = visualization_msgs::msg::Marker::CUBE;
        test_point_mk.ns = "test_point";
        test_point_mk.id = 1;
        test_point_mk.scale.x = 0.01;
        test_point_mk.scale.y = 0.01;
        test_point_mk.scale.z = 0.01;
        test_point_mk.color.a = 1.0;
        test_point_mk.color.g = 1.0;
        test_point_mk.pose.orientation.w = 1;
        test_point_mk.pose = test_point.pose;

        transformed = true;
    }

    RCLCPP_INFO(this->get_logger(), "%d, %d, %d", fl_leg_angles.at(0), fl_leg_angles.at(1), fl_leg_angles.at(2));

    msg.data = {fl_leg_angles.at(0), fl_leg_angles.at(1), fl_leg_angles.at(2), 0, 0, 0, 0, 0, 0, 0, 0, 0};

    test_point_mk_pub->publish(test_point_mk);
    test_pub->publish(msg);
}