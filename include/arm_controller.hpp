#pragma once

#include "arm_dynamics.hpp"
#include "arm_kinematics.hpp"
#include "common_types.hpp"

struct JointSpaceControlGains {
    JointMatrix2d kp;
    JointMatrix2d kd;

    static JointSpaceControlGains diagonal(double kp1, double kp2, double kd1, double kd2);
};

struct TaskSpaceControlGains {
    JointMatrix2d kp;
    JointMatrix2d kd;

    static TaskSpaceControlGains diagonal(double kpx, double kpy, double kdx, double kdy);
};

struct JointSpaceTarget {
    JointVector2d position;
    JointVector2d velocity;
};

struct TaskSpaceTarget {
    TaskVector2d position;
    TaskVector2d velocity;
};

struct ComputedTorqueTarget {
    JointVector2d position;
    JointVector2d velocity;
    JointVector2d acceleration;
};

class ArmController {
public:
    ArmController(const ArmKinematics& kinematics, const ArmDynamics& dynamics);

    JointVector2d jointSpacePdGravity(const JointVector2d& q,
                                      const JointVector2d& v,
                                      const JointSpaceTarget& target,
                                      const JointSpaceControlGains& gains) const;

    JointVector2d taskSpaceJacobianTranspose(const JointVector2d& q,
                                             const JointVector2d& v,
                                             const TaskSpaceTarget& target,
                                             const TaskSpaceControlGains& gains) const;

    JointVector2d computedTorque(const JointVector2d& q,
                                 const JointVector2d& v,
                                 const ComputedTorqueTarget& target,
                                 const JointSpaceControlGains& gains) const;

private:
    const ArmKinematics& kinematics_;
    const ArmDynamics& dynamics_;
};
