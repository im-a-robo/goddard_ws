#pragma once

#include <math.h>

#include <tuple>
#include <vector>

#include "stdio.h"

class KinematicsInterface {
private:
    double hip_to_femur_dist;
    double femur_to_tibia_dist;
    double tibia_to_foot_dist;

    std::vector<double>* leg_angles;

    double calc_hip_joint_delta(double z, double y);

    std::tuple<double, double> calc_femur_and_tibia_joint_delta(double x, double y);

    double sqr(double val) { return pow(val, 2); }

public:
    KinematicsInterface();
    KinematicsInterface(double hip_to_femur_dist, double femur_to_tibia_dist, double tibia_to_foot_dist,
                        std::vector<double>* leg_angles);

    void calc_joint_deltas(double x, double y, double z);
};
