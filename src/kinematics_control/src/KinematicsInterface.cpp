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
    double hypot = sqrt(pow(z, 2) + pow(y, 2));

    double alpha = acos(abs(z) / hypot);
    double beta = acos(hip_to_femur_dist / hypot);

    double q1 = alpha - beta ? y >= 0 : M_PI - alpha - beta;

    return q1;
}

std::tuple<double, double> KinematicsInterface::calc_femur_and_tibia_joint_delta(double x, double y) {
    double hypot = sqrt(pow(x, 2) + pow(y, 2));
    double phi = acos(abs(x) / hypot);
    double lowercase_phi = acos((pow(femur_to_tibia_dist, 2) + pow(x, 2) + pow(y, 2) - pow(tibia_to_foot_dist, 2)) /
                                (2 * femur_to_tibia_dist * hypot));

    // Assume that q3 will always be greater than zero for this robot
    double q2, q3;

    if (x > 0) {
        q2 = M_PI_2 - lowercase_phi - phi;
    } else {
        q2 = -M_PI_2 - lowercase_phi + phi;
    }
    q3 = acos((pow(femur_to_tibia_dist, 2) + pow(tibia_to_foot_dist, 2) - pow(x, 2) - pow(y, 2)) /
              (2 * femur_to_tibia_dist * tibia_to_foot_dist));

    return {q2, q3};
}

void KinematicsInterface::calc_joint_deltas(double x, double y, double z) {
    double q1 = calc_hip_joint_delta(y, z);

    double y_prime = -sqrt(pow(y, 2) + pow(z, 2) - pow(hip_to_femur_dist, 2));

    auto [q2, q3] = calc_femur_and_tibia_joint_delta(x, y_prime);

    leg_angles->at(0) = 0.5;
    leg_angles->at(1) = 0.5;
    leg_angles->at(2) = 1.57;
}