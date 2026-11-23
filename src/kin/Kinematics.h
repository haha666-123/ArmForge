#pragma once

#include <vector>

#include "core/Math.h"
#include "core/Status.h"
#include "model/ArmModel.h"

namespace af {

struct Pose {
    Vec3 position{};
    Mat3 rotation = Mat3::identity();
};

[[nodiscard]] Pose end_effector_pose(const ArmModel& model, const std::vector<double>& q);

// Geometric Jacobian: 6 rows (linear xyz, angular xyz) by dof columns, expressed
// in the base frame.
[[nodiscard]] DenseMatrix geometric_jacobian(const ArmModel& model,
                                             const std::vector<double>& q);

struct IkOptions {
    int max_iterations{200};
    double position_tolerance{1e-4};     // metres
    double orientation_tolerance{1e-3};  // radians
    double damping{0.05};                // Levenberg-Marquardt lambda
    double max_step{0.5};                // rad per iteration
    bool position_only{false};           // ignore orientation (pick and place tasks)
};

struct IkResult {
    bool ok{false};
    std::vector<double> q;
    int iterations{0};
    double position_error{0.0};
    double orientation_error{0.0};
    Status status;
};

// Damped least squares inverse kinematics with joint limit clamping.
//
// The solver is iterative on purpose: it works for any DH chain (no closed form
// per robot), keeps a configurable seed so a follow-up call stays on the same
// solution branch, and reports the residual error instead of pretending success.
[[nodiscard]] IkResult solve_ik(const ArmModel& model, const Pose& target,
                                const std::vector<double>& seed,
                                const IkOptions& options = {});

// Local minima are real for damped least squares: the same target can be
// unreachable from one seed and a two iteration solve from another. This wrapper
// tries the caller seed first and then a few base rotations, returning the best
// solution found instead of failing on the first attempt.
[[nodiscard]] IkResult solve_ik_best(const ArmModel& model, const Pose& target,
                                     const std::vector<double>& seed,
                                     const IkOptions& options = {});

}  // namespace af
