#pragma once

#include "common_types.hpp"

class ArmKinematics {
public:
    ArmKinematics(double link1_length, double link2_length);

    TaskVector2d forwardKinematics(const JointVector2d& q) const;
    Jacobian2d jacobian(const JointVector2d& q) const;

    double link1Length() const;
    double link2Length() const;

private:
    double l1_;
    double l2_;
};
