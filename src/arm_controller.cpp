#include "arm_controller.hpp"

#include <stdexcept>

namespace {

void validateNonNegativeDiagonal(const JointMatrix2d& matrix, const char* name) {
    if (matrix(0, 0) < 0.0 || matrix(1, 1) < 0.0) {
        throw std::invalid_argument(name);
    }
}

}  // namespace

JointSpaceControlGains JointSpaceControlGains::diagonal(double kp1,
                                                        double kp2,
                                                        double kd1,
                                                        double kd2) {
    JointSpaceControlGains gains;
    gains.kp = JointMatrix2d::Zero();
    gains.kd = JointMatrix2d::Zero();
    gains.kp.diagonal() << kp1, kp2;
    gains.kd.diagonal() << kd1, kd2;

    validateNonNegativeDiagonal(gains.kp, "joint-space proportional gains must be non-negative");
    validateNonNegativeDiagonal(gains.kd, "joint-space derivative gains must be non-negative");
    return gains;
}

TaskSpaceControlGains TaskSpaceControlGains::diagonal(double kpx,
                                                      double kpy,
                                                      double kdx,
                                                      double kdy) {
    TaskSpaceControlGains gains;
    gains.kp = JointMatrix2d::Zero();
    gains.kd = JointMatrix2d::Zero();
    gains.kp.diagonal() << kpx, kpy;
    gains.kd.diagonal() << kdx, kdy;

    validateNonNegativeDiagonal(gains.kp, "task-space proportional gains must be non-negative");
    validateNonNegativeDiagonal(gains.kd, "task-space derivative gains must be non-negative");
    return gains;
}

ArmController::ArmController(const ArmKinematics& kinematics, const ArmDynamics& dynamics)
    : kinematics_(kinematics), dynamics_(dynamics) {}

JointVector2d ArmController::jointSpacePdGravity(const JointVector2d& q,
                                                 const JointVector2d& v,
                                                 const JointSpaceTarget& target,
                                                 const JointSpaceControlGains& gains) const {
    validateNonNegativeDiagonal(gains.kp, "joint-space proportional gains must be non-negative");
    validateNonNegativeDiagonal(gains.kd, "joint-space derivative gains must be non-negative");

    return gains.kp * (target.position - q) +
           gains.kd * (target.velocity - v) +
           dynamics_.gravityTerms(q);
}

JointVector2d ArmController::taskSpaceJacobianTranspose(const JointVector2d& q,
                                                        const JointVector2d& v,
                                                        const TaskSpaceTarget& target,
                                                        const TaskSpaceControlGains& gains) const {
    validateNonNegativeDiagonal(gains.kp, "task-space proportional gains must be non-negative");
    validateNonNegativeDiagonal(gains.kd, "task-space derivative gains must be non-negative");

    const TaskVector2d position = kinematics_.forwardKinematics(q);
    const Jacobian2d jacobian = kinematics_.jacobian(q);
    const TaskVector2d velocity = jacobian * v;
    const TaskVector2d task_force = gains.kp * (target.position - position) +
                                    gains.kd * (target.velocity - velocity);

    return jacobian.transpose() * task_force + dynamics_.gravityTerms(q);
}

JointVector2d ArmController::computedTorque(const JointVector2d& q,
                                            const JointVector2d& v,
                                            const ComputedTorqueTarget& target,
                                            const JointSpaceControlGains& gains) const {
    validateNonNegativeDiagonal(gains.kp, "joint-space proportional gains must be non-negative");
    validateNonNegativeDiagonal(gains.kd, "joint-space derivative gains must be non-negative");

    const JointVector2d commanded_acceleration = target.acceleration +
                                                 gains.kd * (target.velocity - v) +
                                                 gains.kp * (target.position - q);

    return dynamics_.inverseDynamics(q, v, commanded_acceleration);
}
