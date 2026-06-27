#pragma once

#include "common_types.hpp"

#include <pinocchio/multibody/model.hpp>

struct ArmDynamicsParameters {
    double link1_length;
    double link2_length;
    double link1_mass;
    double link2_mass;
    double link1_inertia;
    double link2_inertia;
    double gravity;

    static ArmDynamicsParameters uniformLinks(double link1_length,
                                              double link2_length,
                                              double link1_mass,
                                              double link2_mass,
                                              double gravity = 9.81);
};

class ArmDynamics {
public:
    explicit ArmDynamics(const ArmDynamicsParameters& parameters);

    JointMatrix2d massMatrix(const JointVector2d& q) const;
    JointVector2d nonlinearEffects(const JointVector2d& q, const JointVector2d& v) const;
    JointVector2d gravityTerms(const JointVector2d& q) const;
    JointVector2d inverseDynamics(const JointVector2d& q,
                                  const JointVector2d& v,
                                  const JointVector2d& a) const;

    const ArmDynamicsParameters& parameters() const;

private:
    ArmDynamicsParameters parameters_;
    pinocchio::Model model_;
};
