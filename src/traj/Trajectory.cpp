#include "traj/Trajectory.h"

#include <algorithm>
#include <cmath>

namespace af {

namespace {

double sign_of(double value) { return value >= 0.0 ? 1.0 : -1.0; }

double trapezoid_duration(double distance, double vmax, double amax, double& t_acc,
                          double& v_peak) {
    if (distance < 1e-12) {
        t_acc = 0.0;
        v_peak = 0.0;
        return 0.0;
    }
    if (vmax <= 1e-9 || amax <= 1e-9) {
        t_acc = 0.0;
        v_peak = 0.0;
        return 0.0;
    }
    if (vmax * vmax / amax >= distance) {
        // Triangular profile: never reaches vmax.
        t_acc = std::sqrt(distance / amax);
        v_peak = amax * t_acc;
        return 2.0 * t_acc;
    }
    t_acc = vmax / amax;
    v_peak = vmax;
    return t_acc + distance / vmax;
}

double quintic_duration(double distance, double vmax) {
    if (distance < 1e-12 || vmax <= 1e-9) return 0.0;
    // A quintic polynomial peaks at 1.875 * distance / duration.
    return 1.875 * distance / vmax;
}

}  // namespace

JointSegment make_segment(double q0, double q1, double vmax, double amax, Profile profile) {
    JointSegment segment;
    segment.q0 = q0;
    segment.q1 = q1;
    segment.vmax = vmax;
    segment.amax = amax;
    segment.profile = profile;

    const double distance = std::fabs(q1 - q0);
    if (profile == Profile::Quintic) {
        segment.duration = quintic_duration(distance, vmax);
        segment.v_peak = distance > 1e-12 && segment.duration > 1e-12
                             ? 1.875 * distance / segment.duration
                             : 0.0;
        segment.t_acc = segment.duration * 0.5;
    } else {
        segment.duration = trapezoid_duration(distance, vmax, amax, segment.t_acc, segment.v_peak);
    }
    return segment;
}

double JointSegment::sample(double t) const {
    const double distance = q1 - q0;
    if (duration <= 1e-12 || std::fabs(distance) < 1e-12) return q0;
    const double clamped = std::clamp(t, 0.0, duration);

    if (profile == Profile::Quintic) {
        const double s = clamped / duration;
        const double blend = s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
        return q0 + distance * blend;
    }

    const double dir = sign_of(distance);
    const double accel_dist = 0.5 * amax * t_acc * t_acc;
    if (clamped <= t_acc) {
        return q0 + dir * 0.5 * amax * clamped * clamped;
    }
    const double cruise_end = duration - t_acc;
    if (clamped <= cruise_end) {
        return q0 + dir * (accel_dist + v_peak * (clamped - t_acc));
    }
    const double remaining = duration - clamped;
    return q1 - dir * 0.5 * amax * remaining * remaining;
}

double JointSegment::velocity(double t) const {
    const double distance = q1 - q0;
    if (duration <= 1e-12 || std::fabs(distance) < 1e-12) return 0.0;
    const double clamped = std::clamp(t, 0.0, duration);

    if (profile == Profile::Quintic) {
        const double s = clamped / duration;
        const double ds = 30.0 * s * s * (1.0 - 2.0 * s + s * s);
        return distance * ds / duration;
    }

    const double dir = sign_of(distance);
    if (clamped <= t_acc) return dir * amax * clamped;
    if (clamped <= duration - t_acc) return dir * v_peak;
    return dir * amax * (duration - clamped);
}

JointTrajectory JointTrajectory::build(const ArmModel& model, const std::vector<double>& q_start,
                                       const std::vector<double>& q_goal,
                                       const TrajectoryOptions& options) {
    JointTrajectory trajectory;
    const std::size_t dof = model.dof();
    if (q_start.size() != dof || q_goal.size() != dof) return trajectory;

    const double scale = options.speed_scale > 1e-6 ? options.speed_scale : 1.0;

    std::vector<JointSegment> raw;
    raw.reserve(dof);
    double longest = 0.0;
    for (std::size_t i = 0; i < dof; ++i) {
        const auto& limit = model.limits()[i];
        JointSegment segment = make_segment(q_start[i], q_goal[i], limit.max_velocity / scale,
                                            limit.max_acceleration / scale, options.profile);
        longest = std::max(longest, segment.duration);
        raw.push_back(segment);
    }

    if (longest <= 1e-9) {
        trajectory.segments_ = raw;
        trajectory.duration_ = 0.0;
        return trajectory;
    }

    // Slow the shorter joints down so all of them arrive together.
    trajectory.segments_ = raw;
    trajectory.duration_ = longest;
    return trajectory;
}

std::vector<double> JointTrajectory::sample(double t) const {
    std::vector<double> q(segments_.size(), 0.0);
    for (std::size_t i = 0; i < segments_.size(); ++i) {
        const JointSegment& segment = segments_[i];
        if (segment.duration <= 1e-12) {
            q[i] = segment.q1;
            continue;
        }
        const double local = duration_ <= 1e-12 ? segment.duration
                                                : t * (segment.duration / duration_);
        q[i] = segment.sample(local);
    }
    return q;
}

std::vector<double> JointTrajectory::velocity(double t) const {
    std::vector<double> v(segments_.size(), 0.0);
    for (std::size_t i = 0; i < segments_.size(); ++i) {
        const JointSegment& segment = segments_[i];
        if (segment.duration <= 1e-12 || duration_ <= 1e-12) continue;
        const double ratio = segment.duration / duration_;
        v[i] = segment.velocity(t * ratio) * ratio;
    }
    return v;
}

JointTrajectory::MotionStats JointTrajectory::stats(double sample_dt) const {
    MotionStats stats;
    if (duration_ <= 1e-9 || sample_dt <= 1e-6) return stats;

    std::vector<double> previous = velocity(0.0);
    for (double t = sample_dt; t <= duration_ + 1e-9; t += sample_dt) {
        const std::vector<double> current = velocity(t);
        for (std::size_t i = 0; i < current.size(); ++i) {
            stats.max_velocity = std::max(stats.max_velocity, std::fabs(current[i]));
            const double acceleration = std::fabs(current[i] - previous[i]) / sample_dt;
            stats.max_acceleration = std::max(stats.max_acceleration, acceleration);
        }
        previous = current;
    }
    return stats;
}

Mat3 interpolate_rotation(const Mat3& start, const Mat3& end, double s) {
    const Mat3 relative = start.transposed() * end;
    const Vec3 rotation = rotation_vector(relative);
    const double angle = rotation.norm();
    if (angle < 1e-9) return start;
    const Mat3 delta = rotation_from_axis_angle(rotation / angle, angle * s);
    return start * delta;
}

CartesianPath plan_linear_move(const ArmModel& model, const Pose& start, const Pose& end,
                               const CartesianOptions& options) {
    CartesianPath path;
    const double distance = (end.position - start.position).norm();
    path.length = distance;

    const double step = options.step > 1e-4 ? options.step : 1e-4;
    const int steps = std::max(2, static_cast<int>(std::ceil(distance / step)));

    std::vector<double> seed(model.dof(), 0.0);
    Pose previous_pose = start;

    for (int i = 0; i <= steps; ++i) {
        const double s = static_cast<double>(i) / static_cast<double>(steps);
        Pose target;
        target.position = start.position + (end.position - start.position) * s;
        target.rotation = interpolate_rotation(start.rotation, end.rotation, s);

        IkOptions ik_options;
        ik_options.max_iterations = 120;
        const IkResult ik = solve_ik_best(model, target, seed, ik_options);
        if (!ik.ok) {
            path.status = Status::error(errc::kUnreachable,
                                        "cartesian waypoint " + std::to_string(i) +
                                            " is unreachable: " + ik.status.message());
            return path;
        }

        double jump = 0.0;
        for (std::size_t j = 0; j < ik.q.size(); ++j) {
            jump = std::max(jump, std::fabs(ik.q[j] - seed[j]));
        }
        path.max_joint_jump = std::max(path.max_joint_jump, jump);
        if (jump > options.max_joint_jump) {
            path.status = Status::error(errc::kLimitViolation,
                                        "joint jump " + std::to_string(jump) +
                                            " rad at waypoint " + std::to_string(i) +
                                            " (near singularity)");
            return path;
        }

        seed = ik.q;
        path.waypoints.push_back(ik.q);
        path.poses.push_back(target);
        previous_pose = target;
    }

    (void)previous_pose;
    path.status = Status::success();
    return path;
}

}  // namespace af
