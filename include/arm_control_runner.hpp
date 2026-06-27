#pragma once

#include "arm_controller.hpp"
#include "common_types.hpp"

enum class ArmControlMode {
    JointSpacePdGravity,
    TaskSpaceJacobianTranspose,
    ComputedTorque
};

struct TorqueLimits {
    JointVector2d lower;
    JointVector2d upper;

    static TorqueLimits symmetric(double joint1_limit, double joint2_limit);
};

struct ArmControlCommand {
    ArmControlMode mode = ArmControlMode::JointSpacePdGravity;
    JointSpaceTarget joint_target{JointVector2d::Zero(), JointVector2d::Zero()};
    TaskSpaceTarget task_target{TaskVector2d::Zero(), TaskVector2d::Zero()};
    ComputedTorqueTarget computed_torque_target{JointVector2d::Zero(),
                                                JointVector2d::Zero(),
                                                JointVector2d::Zero()};
    JointSpaceControlGains joint_gains{JointMatrix2d::Zero(), JointMatrix2d::Zero()};
    TaskSpaceControlGains task_gains{JointMatrix2d::Zero(), JointMatrix2d::Zero()};
};

class ArmControlRunner {
public:
    ArmControlRunner(const ArmController& controller, const TorqueLimits& torque_limits);

    JointVector2d update(const JointVector2d& q,
                         const JointVector2d& v,
                         const ArmControlCommand& command) const;

private:
    const ArmController& controller_;
    TorqueLimits torque_limits_;
};
