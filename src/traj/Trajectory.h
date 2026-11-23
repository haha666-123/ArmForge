#pragma once

#include <vector>

#include "core/Math.h"
#include "core/Status.h"
#include "kin/Kinematics.h"
#include "model/ArmModel.h"

namespace af {

enum class Profile { Trapezoid, Quintic };

// Single joint move profile. Duration includes acceleration and deceleration.
struct JointSegment {
    double q0{0.0};
    double q1{0.0};
    double vmax{0.0};
    double amax{0.0};
    double duration{0.0};
    double t_acc{0.0};
    double v_peak{0.0};
    Profile profile{Profile::Trapezoid};

    [[nodiscard]] double sample(double t) const;
    [[nodiscard]] double velocity(double t) const;
};

[[nodiscard]] JointSegment make_segment(double q0, double q1, double vmax, double amax,
                                        Profile profile);

struct TrajectoryOptions {
    double speed_scale{1.0};  // > 1 faster, < 1 slower
    Profile profile{Profile::Trapezoid};
};

// Synchronised joint space trajectory: every joint finishes at the same time.
class JointTrajectory {
public:
    JointTrajectory() = default;

    [[nodiscard]] static JointTrajectory build(const ArmModel& model,
                                                const std::vector<double>& q_start,
                                                const std::vector<double>& q_goal,
                                                const TrajectoryOptions& options = {});

    [[nodiscard]] double duration() const noexcept { return duration_; }
    [[nodiscard]] std::size_t dof() const noexcept { return segments_.size(); }

    [[nodiscard]] std::vector<double> sample(double t) const;
    [[nodiscard]] std::vector<double> velocity(double t) const;

    // Peak absolute joint velocity and acceleration over the sampled trajectory.
    struct MotionStats {
        double max_velocity{0.0};
        double max_acceleration{0.0};
    };
    [[nodiscard]] MotionStats stats(double sample_dt = 0.002) const;

private:
    std::vector<JointSegment> segments_;
    double duration_{0.0};
};

struct CartesianOptions {
    double step{0.005};       // metres between IK seeds
    double max_joint_jump{0.35};  // rad between two waypoints, flags a singularity
};

struct CartesianPath {
    std::vector<std::vector<double>> waypoints;
    std::vector<Pose> poses;
    double max_joint_jump{0.0};
    double length{0.0};
    Status status;
};

// Straight line move in Cartesian space, solved point by point with the previous
// solution as seed so the arm stays on one IK branch.
[[nodiscard]] CartesianPath plan_linear_move(const ArmModel& model, const Pose& start,
                                             const Pose& end,
                                             const CartesianOptions& options = {});

[[nodiscard]] Mat3 interpolate_rotation(const Mat3& start, const Mat3& end, double s);

}  // namespace af
