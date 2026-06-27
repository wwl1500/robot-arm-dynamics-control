#include "arm_control_runner.hpp"

#include <stdexcept>

namespace {

void validateTorqueLimits(const TorqueLimits& torque_limits) {
    if ((torque_limits.lower.array() > torque_limits.upper.array()).any()) {
        throw std::invalid_argument("torque limit lower bounds must not exceed upper bounds");
    }
}

JointVector2d clampTorque(const JointVector2d& tau, const TorqueLimits& torque_limits) {
    return tau.cwiseMax(torque_limits.lower).cwiseMin(torque_limits.upper);
}

}  // namespace

TorqueLimits TorqueLimits::symmetric(double joint1_limit, double joint2_limit) {
    if (joint1_limit < 0.0 || joint2_limit < 0.0) {
        throw std::invalid_argument("symmetric torque limits must be non-negative");
    }

    TorqueLimits torque_limits;
    torque_limits.lower << -joint1_limit, -joint2_limit;
    torque_limits.upper << joint1_limit, joint2_limit;
    return torque_limits;
}

ArmControlRunner::ArmControlRunner(const ArmController& controller, const TorqueLimits& torque_limits)
    : controller_(controller), torque_limits_(torque_limits) {
    validateTorqueLimits(torque_limits_);
}

JointVector2d ArmControlRunner::update(const JointVector2d& q,
                                        const JointVector2d& v,
                                        const ArmControlCommand& command) const {
    JointVector2d tau;

    switch (command.mode) {
        case ArmControlMode::JointSpacePdGravity:
            tau = controller_.jointSpacePdGravity(q, v, command.joint_target, command.joint_gains);
            break;
        case ArmControlMode::TaskSpaceJacobianTranspose:
            tau = controller_.taskSpaceJacobianTranspose(q, v, command.task_target, command.task_gains);
            break;
        case ArmControlMode::ComputedTorque:
            tau = controller_.computedTorque(q, v, command.computed_torque_target, command.joint_gains);
            break;
        default:
            throw std::invalid_argument("unknown arm control mode");
    }

    return clampTorque(tau, torque_limits_);
}
