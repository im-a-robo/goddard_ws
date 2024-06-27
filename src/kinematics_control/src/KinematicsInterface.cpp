#include "kinematics_control/KinematicsInterface.hpp"

KinematicsInterface::KinematicsInterface() {}

KinematicsInterface::KinematicsInterface(double hip_to_femur_dist, double femur_to_tibia_dist,
                                         double tibia_to_foot_dist, std::vector<double>* leg_angles) {
    this->hip_to_femur_dist = hip_to_femur_dist;
    this->femur_to_tibia_dist = femur_to_tibia_dist;
    this->tibia_to_foot_dist = tibia_to_foot_dist;

    this->leg_angles = leg_angles;
    leg_angles->at(0) = 0;
    leg_angles->at(1) = 0;
    leg_angles->at(2) = 0;
}

double KinematicsInterface::calc_hip_joint_delta(double y, double z) {
    double hypot = sqrt(sqr(z) + sqr(y));

    double alpha1 = acos(hip_to_femur_dist / hypot);
    double alpha2 = acos(y / hypot);

    return alpha1 - alpha2;
}

std::tuple<double, double> KinematicsInterface::calc_femur_and_tibia_joint_delta(double x, double z) {
    double hypot = sqrt(sqr(x) + sqr(z));

    double phi = acos(abs(x) / hypot);
    double lowercase_phi = acos((sqr(femur_to_tibia_dist) + sqr(x) + sqr(z) - sqr(tibia_to_foot_dist)) /
                                (2 * femur_to_tibia_dist * hypot));

    // Assume that q3 will always be greater than zero for this robot
    double q2, q3;

    if (x > 0) {
        q2 = M_PI_2 + lowercase_phi - phi;
    } else {
        q2 = -M_PI_2 + lowercase_phi + phi;
    }

    q3 = -acos((sqr(femur_to_tibia_dist) + sqr(tibia_to_foot_dist) - sqr(x) - sqr(z)) /
               (2 * femur_to_tibia_dist * tibia_to_foot_dist));

    return {q2, q3};
}

void KinematicsInterface::calc_joint_deltas(double x, double y, double z) {
    double q1 = calc_hip_joint_delta(y, z);

    double z_prime = y * sin(q1) + z * cos(q1);

    auto [q2, q3] = calc_femur_and_tibia_joint_delta(x, z_prime);

    leg_angles->at(0) = q1;
    leg_angles->at(1) = q2;
    leg_angles->at(2) = q3;
}