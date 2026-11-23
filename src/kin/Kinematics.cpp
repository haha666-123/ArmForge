#include "kin/Kinematics.h"

#include <algorithm>
#include <cmath>

namespace af {

Pose end_effector_pose(const ArmModel& model, const std::vector<double>& q) {
    const Mat4 transform = model.end_effector(q);
    Pose pose;
    pose.position = transform.translation();
    pose.rotation = transform.rotation();
    return pose;
}

DenseMatrix geometric_jacobian(const ArmModel& model, const std::vector<double>& q) {
    const auto frames = model.link_frames(q);
    const std::size_t dof = model.dof();
    const Vec3 end = frames.back().translation();

    DenseMatrix jacobian(6, dof);
    for (std::size_t i = 0; i < dof; ++i) {
        const Vec3 axis = frames[i].rotation().col(2).normalized();
        const Vec3 origin = frames[i].translation();
        const Vec3 linear = cross(axis, end - origin);
        jacobian(0, i) = linear.x;
        jacobian(1, i) = linear.y;
        jacobian(2, i) = linear.z;
        jacobian(3, i) = axis.x;
        jacobian(4, i) = axis.y;
        jacobian(5, i) = axis.z;
    }
    return jacobian;
}

IkResult solve_ik(const ArmModel& model, const Pose& target, const std::vector<double>& seed,
                  const IkOptions& options) {
    IkResult result;
    const std::size_t dof = model.dof();

    if (seed.size() != dof) {
        result.status = Status::error(errc::kInvalidArgument,
                                      "seed size " + std::to_string(seed.size()) +
                                          " does not match dof " + std::to_string(dof));
        result.q = std::vector<double>(dof, 0.0);
        return result;
    }

    std::vector<double> q = model.clamp_to_limits(seed);
    double best_position = 1e9;
    double best_orientation = 1e9;
    std::vector<double> best_q = q;

    for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
        const Pose current = end_effector_pose(model, q);
        const Vec3 position_error = target.position - current.position;
        const Vec3 orientation_error =
            rotation_vector(target.rotation * current.rotation.transposed());

        const double position_norm = position_error.norm();
        const double orientation_norm = orientation_error.norm();
        if (position_norm < best_position) {
            best_position = position_norm;
            best_orientation = orientation_norm;
            best_q = q;
        }

        const bool orientation_ok =
            options.position_only || orientation_norm <= options.orientation_tolerance;
        if (position_norm <= options.position_tolerance && orientation_ok) {
            result.ok = true;
            result.q = q;
            result.iterations = iteration;
            result.position_error = position_norm;
            result.orientation_error = orientation_norm;
            result.status = Status::success();
            return result;
        }

        const DenseMatrix jacobian_full = geometric_jacobian(model, q);
        DenseMatrix jacobian = jacobian_full;
        if (options.position_only) {
            DenseMatrix reduced(3, dof);
            for (std::size_t r = 0; r < 3; ++r) {
                for (std::size_t c = 0; c < dof; ++c) reduced(r, c) = jacobian_full(r, c);
            }
            jacobian = reduced;
        }
        const DenseMatrix jacobian_t = jacobian.transposed();

        // Levenberg-Marquardt step: dq = J^T (J J^T + lambda^2 I)^-1 e
        DenseMatrix normal = jacobian * jacobian_t;
        normal.add_to_diagonal(options.damping * options.damping);

        std::vector<double> error{position_error.x, position_error.y, position_error.z};
        if (!options.position_only) {
            error.push_back(orientation_error.x);
            error.push_back(orientation_error.y);
            error.push_back(orientation_error.z);
        }

        std::vector<double> y = solve_linear(normal, error);
        if (y.empty()) {
            // Singular configuration: increase damping once and retry.
            DenseMatrix fallback = jacobian * jacobian_t;
            fallback.add_to_diagonal(10.0 * options.damping * options.damping);
            y = solve_linear(fallback, error);
            if (y.empty()) break;
        }

        DenseMatrix y_matrix(y.size(), 1);
        for (std::size_t i = 0; i < y.size(); ++i) y_matrix(i, 0) = y[i];
        const DenseMatrix dq_matrix = jacobian_t * y_matrix;

        double step_norm = 0.0;
        std::vector<double> dq(dof, 0.0);
        for (std::size_t i = 0; i < dof; ++i) {
            dq[i] = dq_matrix(i, 0);
            step_norm += dq[i] * dq[i];
        }
        step_norm = std::sqrt(step_norm);
        const double scale = step_norm > options.max_step ? options.max_step / step_norm : 1.0;

        for (std::size_t i = 0; i < dof; ++i) q[i] += dq[i] * scale;
        q = model.clamp_to_limits(q);
    }

    result.ok = false;
    result.q = best_q;
    result.iterations = options.max_iterations;
    result.position_error = best_position;
    result.orientation_error = best_orientation;
    result.status = Status::error(errc::kNoConvergence,
                                  "ik did not converge: pos=" + std::to_string(best_position) +
                                      " rot=" + std::to_string(best_orientation));
    return result;
}

IkResult solve_ik_best(const ArmModel& model, const Pose& target, const std::vector<double>& seed,
                       const IkOptions& options) {
    IkResult best = solve_ik(model, target, seed, options);
    if (best.ok) return best;

    const std::size_t dof = model.dof();
    std::vector<double> base_seed = seed.size() == dof ? seed : std::vector<double>(dof, 0.0);
    const std::array<double, 3> base_offsets{af::kPi / 2.0, -af::kPi / 2.0, af::kPi};

    for (const double offset : base_offsets) {
        std::vector<double> candidate = base_seed;
        if (dof > 0) {
            const auto& limit = model.limits()[0];
            candidate[0] = std::clamp(candidate[0] + offset, limit.min, limit.max);
        }
        const IkResult attempt = solve_ik(model, target, candidate, options);
        if (attempt.ok) return attempt;
        if (attempt.position_error < best.position_error) best = attempt;
    }
    return best;
}

}  // namespace af


