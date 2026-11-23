#include "test_framework.h"

#include "control/ServoSim.h"
#include "model/ArmModel.h"
#include "traj/Trajectory.h"

using namespace af;

TEST(servo_tracks_planned_trajectory) {
    const ArmModel model = ArmModel::ur5_like();
    const auto home = ArmModel::home_configuration();
    std::vector<double> goal = home;
    goal[1] -= 0.5;
    goal[2] += 0.3;

    const JointTrajectory trajectory = JointTrajectory::build(model, home, goal);
    const ServoReport report = simulate_servo(model, trajectory);

    CHECK(report.status.ok());
    CHECK(!report.steps.empty());
    CHECK(report.max_error < 0.1);
    CHECK(report.rms_error < 0.05);
    CHECK(report.within_limits);
    CHECK(report.steps.size() > 20);
}

TEST(servo_reports_velocity_limit) {
    const ArmModel model = ArmModel::ur5_like();
    const auto home = ArmModel::home_configuration();
    std::vector<double> goal = home;
    goal[0] += 2.0;

    const JointTrajectory trajectory = JointTrajectory::build(model, home, goal);
    ServoConfig config;
    config.velocity_scale = 0.5;
    const ServoReport report = simulate_servo(model, trajectory, config);

    CHECK(report.status.ok());
    for (const auto& step : report.steps) {
        for (std::size_t i = 0; i < step.velocity.size(); ++i) {
            CHECK(std::fabs(step.velocity[i]) <= model.limits()[i].max_velocity * 0.5 + 1e-6);
        }
    }
}

TEST(servo_rejects_bad_rate) {
    const ArmModel model = ArmModel::ur5_like();
    const JointTrajectory trajectory;
    ServoConfig config;
    config.rate_hz = 0.0;

    const ServoReport report = simulate_servo(model, trajectory, config);
    CHECK(!report.status.ok());
}
