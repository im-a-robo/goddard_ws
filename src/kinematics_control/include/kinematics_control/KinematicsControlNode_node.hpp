#pragma once

#include "KinematicsInterface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

class KinematicsControlNode : public rclcpp::Node {
private:
    void test_cb();

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr test_pub;
    std::shared_ptr<rclcpp::WallTimer<std::_Bind<void (KinematicsControlNode::*(KinematicsControlNode*))()>>> timer;

    std::vector<double> fl_leg_angles;

    KinematicsInterface interface;

    geometry_msgs::msg::PoseStamped test_point;

    visualization_msgs::msg::Marker test_point_mk;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr test_point_mk_pub;

    bool transformed;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

public:
    KinematicsControlNode(const rclcpp::NodeOptions& options);
};