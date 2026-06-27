#include "arm_control_runner.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

bool checkVectorNear(const std::string& name,
                     const Eigen::Vector2d& actual,
                     const Eigen::Vector2d& expected,
                     double tolerance) {
    const double error = (actual - expected).norm();
    if (error > tolerance) {
        std::cerr << "[FAIL] " << name << '\n'
                  << "actual:   " << actual.transpose() << '\n'
                  << "expected: " << expected.transpose() << '\n'
                  << "error:    " << error << '\n';
        return false;
    }

    std::cout << "[PASS] " << name << '\n';
    return true;
}

template <typename Operation>
bool checkThrowsInvalidArgument(const std::string& name, Operation operation) {
    try {
        operation();
    } catch (const std::invalid_argument&) {
        std::cout << "[PASS] " << name << '\n';
        return true;
    }

    std::cerr << "[FAIL] " << name << '\n';
    return false;
}

}  // namespace

int main() {
    const ArmKinematics kinematics(1.0, 1.0);
    const ArmDynamics dynamics(ArmDynamicsParameters::uniformLinks(1.0, 1.0, 1.0, 1.0));
    const ArmController controller(kinematics, dynamics);
    const ArmControlRunner runner(controller, TorqueLimits::symmetric(100.0, 100.0));
    const double exact_tolerance = 1e-9;

    bool passed = true;

    JointVector2d q;
    q << 0.3, -0.7;
    JointVector2d v;
    v << 0.2, -0.1;

    ArmControlCommand command;
    command.joint_gains = JointSpaceControlGains::diagonal(10.0, 20.0, 1.0, 2.0);
    command.task_gains = TaskSpaceControlGains::diagonal(30.0, 40.0, 3.0, 4.0);
    command.joint_target.position << 0.5, -0.2;
    command.joint_target.velocity << 0.0, 0.0;
    command.task_target.position = kinematics.forwardKinematics(q) + (TaskVector2d() << 0.1, -0.2).finished();
    command.task_target.velocity.setZero();
    command.computed_torque_target.position << 0.4, -0.5;
    command.computed_torque_target.velocity << 0.0, 0.0;
    command.computed_torque_target.acceleration << 0.1, -0.2;

    command.mode = ArmControlMode::JointSpacePdGravity;
    passed &= checkVectorNear("joint-space mode dispatch",
                              runner.update(q, v, command),
                              controller.jointSpacePdGravity(q, v, command.joint_target, command.joint_gains),
                              exact_tolerance);

    command.mode = ArmControlMode::TaskSpaceJacobianTranspose;
    passed &= checkVectorNear("task-space mode dispatch",
                              runner.update(q, v, command),
                              controller.taskSpaceJacobianTranspose(q, v, command.task_target, command.task_gains),
                              exact_tolerance);

    command.mode = ArmControlMode::ComputedTorque;
    passed &= checkVectorNear("computed-torque mode dispatch",
                              runner.update(q, v, command),
                              controller.computedTorque(q, v, command.computed_torque_target, command.joint_gains),
                              exact_tolerance);

    const TorqueLimits symmetric_limits = TorqueLimits::symmetric(1.5, 2.5);
    JointVector2d expected_lower;
    expected_lower << -1.5, -2.5;
    JointVector2d expected_upper;
    expected_upper << 1.5, 2.5;
    passed &= checkVectorNear("symmetric torque limit lower bounds",
                              symmetric_limits.lower,
                              expected_lower,
                              exact_tolerance);
    passed &= checkVectorNear("symmetric torque limit upper bounds",
                              symmetric_limits.upper,
                              expected_upper,
                              exact_tolerance);

    TorqueLimits asymmetric_limits;
    asymmetric_limits.lower << -0.5, -2.0;
    asymmetric_limits.upper << 0.75, 1.0;
    const ArmControlRunner limited_runner(controller, asymmetric_limits);
    command.mode = ArmControlMode::ComputedTorque;
    const JointVector2d raw_tau = controller.computedTorque(q, v, command.computed_torque_target, command.joint_gains);
    const JointVector2d expected_limited_tau = raw_tau.cwiseMax(asymmetric_limits.lower).cwiseMin(asymmetric_limits.upper);
    passed &= checkVectorNear("torque output is clamped per joint",
                              limited_runner.update(q, v, command),
                              expected_limited_tau,
                              exact_tolerance);

    passed &= checkThrowsInvalidArgument("negative symmetric torque limits are rejected", []() {
        (void)TorqueLimits::symmetric(-1.0, 1.0);
    });

    passed &= checkThrowsInvalidArgument("inverted torque limits are rejected", [&controller]() {
        TorqueLimits invalid_limits;
        invalid_limits.lower << 1.0, -1.0;
        invalid_limits.upper << 0.0, 1.0;
        const ArmControlRunner invalid_runner(controller, invalid_limits);
        (void)invalid_runner;
    });

    if (!passed) {
        return 1;
    }

    std::cout << "All control runner checks passed.\n";
    return 0;
}
