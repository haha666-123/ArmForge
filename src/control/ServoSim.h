#pragma once

#include <vector>

#include "core/Math.h"
#include "core/Status.h"
#include "model/ArmModel.h"
#include "traj/Trajectory.h"

namespace af {

struct ServoConfig {
    double rate_hz{200.0};        // control cycle
    double kp{40.0};              // proportional gain (1/s)
    double velocity_scale{1.0};   // <=1 derates the joint velocity limits
    double acceleration_scale{1.0};
};

struct ServoStep {
    double time{0.0};
    std::vector<double> desired;
    std::vector<double> actual;
    std::vector<double> error;
    std::vector<double> velocity;
};

struct ServoReport {
    std::vector<ServoStep> steps;
    double max_error{0.0};        // rad
    double rms_error{0.0};        // rad
    double max_velocity{0.0};     // rad/s
    double max_acceleration{0.0}; // rad/s^2
    bool within_limits{true};
    Status status;
};

// Discrete servo loop: the controller follows a planned trajectory with a P
// term, velocity and acceleration saturation, and joint limit clamping. Used to
// check that a plan is trackable before it ever reaches real hardware.
[[nodiscard]] ServoReport simulate_servo(const ArmModel& model, const JointTrajectory& trajectory,
                                         const ServoConfig& config = {});

}  // namespace af
