#include "arm_kinematics.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>

namespace {

constexpr double kPi = 3.14159265358979323846;

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

bool checkMatrixNear(const std::string& name,
                     const Eigen::Matrix2d& actual,
                     const Eigen::Matrix2d& expected,
                     double tolerance) {
    const double error = (actual - expected).norm();
    if (error > tolerance) {
        std::cerr << "[FAIL] " << name << '\n'
                  << "actual:\n" << actual << '\n'
                  << "expected:\n" << expected << '\n'
                  << "error: " << error << '\n';
        return false;
    }

    std::cout << "[PASS] " << name << '\n';
    return true;
}

Eigen::Matrix2d finiteDifferenceJacobian(const ArmKinematics& kinematics,
                                         const Eigen::Vector2d& q,
                                         double epsilon) {
    Eigen::Matrix2d numeric_jacobian;

    for (int i = 0; i < q.size(); ++i) {
        Eigen::Vector2d q_plus = q;
        Eigen::Vector2d q_minus = q;
        q_plus(i) += epsilon;
        q_minus(i) -= epsilon;

        numeric_jacobian.col(i) =
            (kinematics.forwardKinematics(q_plus) - kinematics.forwardKinematics(q_minus)) /
            (2.0 * epsilon);
    }

    return numeric_jacobian;
}

}  // namespace

int main() {
    const ArmKinematics kinematics(1.0, 1.0);
    const double exact_tolerance = 1e-9;
    const double finite_difference_tolerance = 1e-6;

    bool passed = true;

    Eigen::Vector2d q;
    Eigen::Vector2d expected_position;
    Eigen::Matrix2d expected_jacobian;

    q << 0.0, 0.0;
    expected_position << 2.0, 0.0;
    expected_jacobian << 0.0, 0.0,
                         2.0, 1.0;
    passed &= checkVectorNear("forward kinematics at q = [0, 0]",
                              kinematics.forwardKinematics(q),
                              expected_position,
                              exact_tolerance);
    passed &= checkMatrixNear("jacobian at q = [0, 0]",
                              kinematics.jacobian(q),
                              expected_jacobian,
                              exact_tolerance);

    q << kPi / 2.0, 0.0;
    expected_position << 0.0, 2.0;
    expected_jacobian << -2.0, -1.0,
                          0.0,  0.0;
    passed &= checkVectorNear("forward kinematics at q = [pi/2, 0]",
                              kinematics.forwardKinematics(q),
                              expected_position,
                              exact_tolerance);
    passed &= checkMatrixNear("jacobian at q = [pi/2, 0]",
                              kinematics.jacobian(q),
                              expected_jacobian,
                              exact_tolerance);

    q << 0.0, kPi / 2.0;
    expected_position << 1.0, 1.0;
    expected_jacobian << -1.0, -1.0,
                          1.0,  0.0;
    passed &= checkVectorNear("forward kinematics at q = [0, pi/2]",
                              kinematics.forwardKinematics(q),
                              expected_position,
                              exact_tolerance);
    passed &= checkMatrixNear("jacobian at q = [0, pi/2]",
                              kinematics.jacobian(q),
                              expected_jacobian,
                              exact_tolerance);

    q << 0.3, -0.7;
    const Eigen::Matrix2d numeric_jacobian = finiteDifferenceJacobian(kinematics, q, 1e-6);
    passed &= checkMatrixNear("finite-difference jacobian check",
                              kinematics.jacobian(q),
                              numeric_jacobian,
                              finite_difference_tolerance);

    if (!passed) {
        return 1;
    }

    std::cout << "All kinematics checks passed.\n";
    return 0;
}
