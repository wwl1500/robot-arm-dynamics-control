#include "arm_kinematics.hpp"

#include <cmath>
#include <stdexcept>

ArmKinematics::ArmKinematics(double link1_length, double link2_length)
    : l1_(link1_length), l2_(link2_length) {
    if (link1_length <= 0.0 || link2_length <= 0.0) {
        throw std::invalid_argument("link lengths must be positive");
    }
}

TaskVector2d ArmKinematics::forwardKinematics(const JointVector2d& q) const {
    const double q1 = q(0);
    const double q2 = q(1);
    const double q12 = q1 + q2;

    TaskVector2d x;
    x(0) = l1_ * std::cos(q1) + l2_ * std::cos(q12);
    x(1) = l1_ * std::sin(q1) + l2_ * std::sin(q12);
    return x;
}

Jacobian2d ArmKinematics::jacobian(const JointVector2d& q) const {
    const double q1 = q(0);
    const double q2 = q(1);
    const double q12 = q1 + q2;

    Jacobian2d J;
    J(0, 0) = -l1_ * std::sin(q1) - l2_ * std::sin(q12);
    J(0, 1) = -l2_ * std::sin(q12);
    J(1, 0) = l1_ * std::cos(q1) + l2_ * std::cos(q12);
    J(1, 1) = l2_ * std::cos(q12);
    return J;
}

double ArmKinematics::link1Length() const {
    return l1_;
}

double ArmKinematics::link2Length() const {
    return l2_;
}
