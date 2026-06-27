#include "arm_dynamics.hpp"

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

bool checkMassMatrixPositiveDefinite(const std::string& name,
                                     const Eigen::Matrix2d& mass_matrix,
                                     double tolerance) {
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(mass_matrix);
    const double min_eigenvalue = solver.eigenvalues().minCoeff();
    if (min_eigenvalue <= tolerance) {
        std::cerr << "[FAIL] " << name << '\n'
                  << "mass matrix:\n" << mass_matrix << '\n'
                  << "min eigenvalue: " << min_eigenvalue << '\n';
        return false;
    }

    std::cout << "[PASS] " << name << '\n';
    return true;
}

}  // namespace

int main() {
    const ArmDynamics dynamics(ArmDynamicsParameters::uniformLinks(1.0, 1.0, 1.0, 1.0));
    const double exact_tolerance = 1e-9;

    bool passed = true;

    Eigen::Vector2d q;
    q << 0.3, -0.7;
    const Eigen::Matrix2d mass_matrix = dynamics.massMatrix(q);
    passed &= checkMatrixNear("mass matrix symmetry",
                              mass_matrix,
                              mass_matrix.transpose(),
                              exact_tolerance);

    passed &= checkMassMatrixPositiveDefinite("mass matrix positive definite at q = [0.3, -0.7]",
                                              mass_matrix,
                                              exact_tolerance);

    q << 0.0, 0.0;
    passed &= checkMassMatrixPositiveDefinite("mass matrix positive definite at q = [0, 0]",
                                              dynamics.massMatrix(q),
                                              exact_tolerance);

    Eigen::Vector2d expected_gravity;
    expected_gravity << 19.62, 4.905;
    passed &= checkVectorNear("gravity terms at q = [0, 0]",
                              dynamics.gravityTerms(q),
                              expected_gravity,
                              exact_tolerance);

    q << kPi / 2.0, 0.0;
    passed &= checkMassMatrixPositiveDefinite("mass matrix positive definite at q = [pi/2, 0]",
                                              dynamics.massMatrix(q),
                                              exact_tolerance);

    expected_gravity << 0.0, 0.0;
    passed &= checkVectorNear("gravity terms at q = [pi/2, 0]",
                              dynamics.gravityTerms(q),
                              expected_gravity,
                              exact_tolerance);

    q << 0.3, -0.7;
    Eigen::Vector2d v;
    v << 0.2, -0.4;
    Eigen::Vector2d a;
    a << 0.5, 0.1;
    passed &= checkVectorNear("inverse dynamics consistency",
                              dynamics.inverseDynamics(q, v, a),
                              dynamics.massMatrix(q) * a + dynamics.nonlinearEffects(q, v),
                              exact_tolerance);

    v.setZero();
    a.setZero();
    passed &= checkVectorNear("inverse dynamics equals gravity at rest",
                              dynamics.inverseDynamics(q, v, a),
                              dynamics.gravityTerms(q),
                              exact_tolerance);
    passed &= checkVectorNear("nonlinear effects equals gravity at zero velocity",
                              dynamics.nonlinearEffects(q, v),
                              dynamics.gravityTerms(q),
                              exact_tolerance);

    if (!passed) {
        return 1;
    }

    std::cout << "All dynamics checks passed.\n";
    return 0;
}
