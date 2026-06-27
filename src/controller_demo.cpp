#include "arm_controller.hpp"

#include <Eigen/Dense>
#include <iostream>
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

}  // namespace

int main() {
    const ArmKinematics kinematics(1.0, 1.0);
    const ArmDynamics dynamics(ArmDynamicsParameters::uniformLinks(1.0, 1.0, 1.0, 1.0));
    const ArmController controller(kinematics, dynamics);
    const double exact_tolerance = 1e-9;

    bool passed = true;

    JointVector2d q;
    q << 0.3, -0.7;
    JointVector2d v;
    v << 0.2, -0.1;

    const JointSpaceControlGains joint_gains = JointSpaceControlGains::diagonal(10.0, 20.0, 1.0, 2.0);
    JointSpaceTarget joint_target;
    joint_target.position << 0.5, -0.2;
    joint_target.velocity << 0.0, 0.0;

    JointVector2d expected_joint_tau = joint_gains.kp * (joint_target.position - q) +
                                       joint_gains.kd * (joint_target.velocity - v) +
                                       dynamics.gravityTerms(q);
    passed &= checkVectorNear("joint-space PD with gravity compensation",
                              controller.jointSpacePdGravity(q, v, joint_target, joint_gains),
                              expected_joint_tau,
                              exact_tolerance);

    joint_target.position = q;
    joint_target.velocity = v;
    passed &= checkVectorNear("joint-space zero error equals gravity compensation",
                              controller.jointSpacePdGravity(q, v, joint_target, joint_gains),
                              dynamics.gravityTerms(q),
                              exact_tolerance);

    const TaskSpaceControlGains task_gains = TaskSpaceControlGains::diagonal(30.0, 40.0, 3.0, 4.0);
    const TaskVector2d position = kinematics.forwardKinematics(q);
    const Jacobian2d jacobian = kinematics.jacobian(q);
    const TaskVector2d velocity = jacobian * v;

    TaskSpaceTarget task_target;
    task_target.position = position + (TaskVector2d() << 0.1, -0.2).finished();
    task_target.velocity.setZero();

    const TaskVector2d task_force = task_gains.kp * (task_target.position - position) +
                                    task_gains.kd * (task_target.velocity - velocity);
    const JointVector2d expected_task_tau = jacobian.transpose() * task_force + dynamics.gravityTerms(q);
    passed &= checkVectorNear("task-space Jacobian transpose control",
                              controller.taskSpaceJacobianTranspose(q, v, task_target, task_gains),
                              expected_task_tau,
                              exact_tolerance);

    const JointSpaceControlGains computed_gains = JointSpaceControlGains::diagonal(10.0, 10.0, 2.0, 2.0);
    ComputedTorqueTarget computed_target;
    computed_target.position << 0.4, -0.5;
    computed_target.velocity << 0.0, 0.0;
    computed_target.acceleration << 0.1, -0.2;

    const JointVector2d commanded_acceleration = computed_target.acceleration +
                                                 computed_gains.kd * (computed_target.velocity - v) +
                                                 computed_gains.kp * (computed_target.position - q);
    passed &= checkVectorNear("computed torque control",
                              controller.computedTorque(q, v, computed_target, computed_gains),
                              dynamics.inverseDynamics(q, v, commanded_acceleration),
                              exact_tolerance);

    computed_target.position = q;
    computed_target.velocity = v;
    computed_target.acceleration.setZero();
    const JointVector2d zero_error_tau = controller.computedTorque(q, v, computed_target, computed_gains);
    passed &= checkVectorNear("computed torque zero error equals inverse dynamics bias",
                              zero_error_tau,
                              dynamics.inverseDynamics(q, v, JointVector2d::Zero()),
                              exact_tolerance);
    passed &= checkVectorNear("computed torque zero error equals nonlinear effects",
                              zero_error_tau,
                              dynamics.nonlinearEffects(q, v),
                              exact_tolerance);

    if (!passed) {
        return 1;
    }

    std::cout << "All controller checks passed.\n";
    return 0;
}
