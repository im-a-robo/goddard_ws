#pragma once

#include <math.h>

#include <unordered_map>

class kinematics_interface {
private:
    double hip_to_femur_dist;
    double femur_to_tibia_dist;
    double tibia_to_foot_dist;

    std::unordered_map<const char*, double> leg_angle_map;

public:
    kinematics_interface(double hip_to_femur_dist, double femur_to_tibia_dist, double tibia_to_foot_dist);

    double calc_hip_joint_delta(double z, double y);

    std::tuple<double, double> calc_femur_and_tibia_joint_delta(double x, double y);

    std::unordered_map<const char*, double> calc_joint_deltas(double x, double y, double z);
};
