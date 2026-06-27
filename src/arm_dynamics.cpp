#include "arm_dynamics.hpp"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/joint/joint-revolute.hpp>

#include <Eigen/Dense>
#include <stdexcept>

namespace {

void validateParameters(const ArmDynamicsParameters& parameters) {
    if (parameters.link1_length <= 0.0 || parameters.link2_length <= 0.0) {
        throw std::invalid_argument("link lengths must be positive");
    }
    if (parameters.link1_mass <= 0.0 || parameters.link2_mass <= 0.0) {
        throw std::invalid_argument("link masses must be positive");
    }
    if (parameters.link1_inertia <= 0.0 || parameters.link2_inertia <= 0.0) {
        throw std::invalid_argument("link inertias must be positive");
    }
    if (parameters.gravity < 0.0) {
        throw std::invalid_argument("gravity must be non-negative");
    }
}

pinocchio::Inertia makePlanarLinkInertia(double mass, double center_of_mass_x, double inertia_z) {
    Eigen::Matrix3d rotational_inertia = Eigen::Matrix3d::Zero();
    rotational_inertia(2, 2) = inertia_z;
    return pinocchio::Inertia(mass, Eigen::Vector3d(center_of_mass_x, 0.0, 0.0), rotational_inertia);
}

pinocchio::Model buildPlanar2RModel(const ArmDynamicsParameters& parameters) {
    validateParameters(parameters);

    pinocchio::Model model;
    model.gravity.linear() = Eigen::Vector3d(0.0, -parameters.gravity, 0.0);

    const pinocchio::JointIndex joint1_id = model.addJoint(
        0,
        pinocchio::JointModelRZ(),
        pinocchio::SE3::Identity(),
        "joint1");
    model.appendBodyToJoint(
        joint1_id,
        makePlanarLinkInertia(parameters.link1_mass,
                              parameters.link1_length / 2.0,
                              parameters.link1_inertia),
        pinocchio::SE3::Identity());

    const pinocchio::SE3 joint2_placement(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(parameters.link1_length, 0.0, 0.0));
    const pinocchio::JointIndex joint2_id = model.addJoint(
        joint1_id,
        pinocchio::JointModelRZ(),
        joint2_placement,
        "joint2");
    model.appendBodyToJoint(
        joint2_id,
        makePlanarLinkInertia(parameters.link2_mass,
                              parameters.link2_length / 2.0,
                              parameters.link2_inertia),
        pinocchio::SE3::Identity());

    return model;
}

}  // namespace

ArmDynamicsParameters ArmDynamicsParameters::uniformLinks(double link1_length,
                                                          double link2_length,
                                                          double link1_mass,
                                                          double link2_mass,
                                                          double gravity) {
    return ArmDynamicsParameters{
        link1_length,
        link2_length,
        link1_mass,
        link2_mass,
        link1_mass * link1_length * link1_length / 12.0,
        link2_mass * link2_length * link2_length / 12.0,
        gravity,
    };
}

ArmDynamics::ArmDynamics(const ArmDynamicsParameters& parameters)
    : parameters_(parameters), model_(buildPlanar2RModel(parameters)) {
    if (model_.nq != 2 || model_.nv != 2) {
        throw std::runtime_error("planar 2R model must have nq = 2 and nv = 2");
    }
}

JointMatrix2d ArmDynamics::massMatrix(const JointVector2d& q) const {
    pinocchio::Data data(model_);
    pinocchio::crba(model_, data, q);
    data.M.triangularView<Eigen::StrictlyLower>() =
        data.M.transpose().triangularView<Eigen::StrictlyLower>();
    return data.M.topLeftCorner<2, 2>();
}

JointVector2d ArmDynamics::nonlinearEffects(const JointVector2d& q, const JointVector2d& v) const {
    pinocchio::Data data(model_);
    return pinocchio::nonLinearEffects(model_, data, q, v).head<2>();
}

JointVector2d ArmDynamics::gravityTerms(const JointVector2d& q) const {
    pinocchio::Data data(model_);
    return pinocchio::computeGeneralizedGravity(model_, data, q).head<2>();
}

JointVector2d ArmDynamics::inverseDynamics(const JointVector2d& q,
                                           const JointVector2d& v,
                                           const JointVector2d& a) const {
    pinocchio::Data data(model_);
    return pinocchio::rnea(model_, data, q, v, a).head<2>();
}

const ArmDynamicsParameters& ArmDynamics::parameters() const {
    return parameters_;
}
