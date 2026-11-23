#include "control/ServoSim.h"

#include <algorithm>
#include <cmath>

namespace af {

ServoReport simulate_servo(const ArmModel& model, const JointTrajectory& trajectory,
                           const ServoConfig& config) {
    ServoReport report;
    const std::size_t dof = model.dof();
    if (trajectory.dof() != dof) {
        report.status = Status::error(errc::kInvalidArgument, "trajectory dof mismatch");
        return report;
    }
    if (config.rate_hz <= 1.0) {
        report.status = Status::error(errc::kInvalidArgument, "rate_hz must be positive");
        return report;
    }

    const double dt = 1.0 / config.rate_hz;
    const double duration = trajectory.duration();

    std::vector<double> q = trajectory.sample(0.0);
    std::vector<double> v(dof, 0.0);

    double squared_error = 0.0;
    std::size_t samples = 0;

    for (double t = 0.0; t <= duration + 1e-9; t += dt) {
        const std::vector<double> desired = trajectory.sample(t);

        ServoStep step;
        step.time = t;
        step.desired = desired;
        step.actual = q;
        step.error.resize(dof);
        step.velocity.resize(dof);

        for (std::size_t i = 0; i < dof; ++i) {
            const auto& limit = model.limits()[i];
            const double velocity_limit = limit.max_velocity * config.velocity_scale;
            const double acceleration_limit = limit.max_acceleration * config.acceleration_scale;

            const double error = desired[i] - q[i];
            step.error[i] = error;
            report.max_error = std::max(report.max_error, std::fabs(error));
            squared_error += error * error;

            double command = config.kp * error;
            command = std::clamp(command, -velocity_limit, velocity_limit);

            // Acceleration limit: the joint cannot change speed instantly.
            const double lower = v[i] - acceleration_limit * dt;
            const double upper = v[i] + acceleration_limit * dt;
            v[i] = std::clamp(command, lower, upper);

            q[i] += v[i] * dt;
            q[i] = std::clamp(q[i], limit.min, limit.max);

            step.velocity[i] = v[i];
            report.max_velocity = std::max(report.max_velocity, std::fabs(v[i]));

            const double previous_speed = std::fabs(v[i] - command);
            report.max_acceleration =
                std::max(report.max_acceleration, previous_speed / dt);

            if (std::fabs(v[i]) > velocity_limit + 1e-6) report.within_limits = false;
            if (std::fabs(q[i] - limit.min) < 1e-9 || std::fabs(q[i] - limit.max) < 1e-9) {
                // Saturated on a joint limit: report it, do not silently ignore.
                if (std::fabs(error) > 1e-3) report.within_limits = false;
            }
        }

        report.steps.push_back(std::move(step));
        samples++;
    }

    report.rms_error = samples > 0 ? std::sqrt(squared_error / static_cast<double>(samples * dof))
                                   : 0.0;
    report.status = Status::success();
    return report;
}

}  // namespace af
