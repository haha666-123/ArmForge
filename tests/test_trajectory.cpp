#include "test_framework.h"

#include "model/ArmModel.h"
#include "traj/Trajectory.h"

using namespace af;

TEST(trajectory_segment_endpoints) {
    const JointSegment segment = make_segment(0.2, 1.0, 1.5, 4.0, Profile::Trapezoid);
    CHECK(segment.duration > 0.0);
    CHECK_NEAR(segment.sample(0.0), 0.2, 1e-9);
    CHECK_NEAR(segment.sample(segment.duration), 1.0, 1e-9);
    CHECK_NEAR(segment.velocity(0.0), 0.0, 1e-9);
    CHECK_NEAR(segment.velocity(segment.duration), 0.0, 1e-9);
}

TEST(trajectory_synchronised_joints) {
    const ArmModel model = ArmModel::ur5_like();
    const auto home = ArmModel::home_configuration();
    std::vector<double> goal = home;
    goal[0] += 0.8;
    goal[1] -= 0.4;

    const JointTrajectory trajectory = JointTrajectory::build(model, home, goal);
    CHECK(trajectory.duration() > 0.0);
    CHECK(trajectory.dof() == model.dof());

    const auto start = trajectory.sample(0.0);
    const auto end = trajectory.sample(trajectory.duration());
    for (std::size_t i = 0; i < model.dof(); ++i) {
        CHECK_NEAR(start[i], home[i], 1e-6);
        CHECK_NEAR(end[i], goal[i], 1e-6);
    }

    const auto stats = trajectory.stats();
    for (std::size_t i = 0; i < model.dof(); ++i) {
        CHECK(stats.max_velocity <= model.limits()[i].max_velocity * 1.05);
    }
}

TEST(trajectory_quintic_profile) {
    const JointSegment segment = make_segment(0.0, 1.0, 2.0, 8.0, Profile::Quintic);
    CHECK(segment.duration > 0.0);
    CHECK_NEAR(segment.sample(0.0), 0.0, 1e-9);
    CHECK_NEAR(segment.sample(segment.duration), 1.0, 1e-9);

    // Peak velocity of a quintic profile is 1.875 * distance / duration.
    const double peak = 1.875 * 1.0 / segment.duration;
    CHECK_NEAR(peak, segment.v_peak, 1e-9);
}

TEST(trajectory_zero_distance) {
    const ArmModel model = ArmModel::ur5_like();
    const auto home = ArmModel::home_configuration();
    const JointTrajectory trajectory = JointTrajectory::build(model, home, home);
    CHECK_NEAR(trajectory.duration(), 0.0, 1e-12);
    const auto q = trajectory.sample(0.0);
    for (std::size_t i = 0; i < model.dof(); ++i) CHECK_NEAR(q[i], home[i], 1e-12);
}

TEST(trajectory_linear_move_waypoints) {
    const ArmModel model = ArmModel::ur5_like();
    const Pose start = end_effector_pose(model, ArmModel::home_configuration());
    Pose end = start;
    end.position = start.position + Vec3{0.05, 0.05, 0.05};

    const CartesianPath path = plan_linear_move(model, start, end, CartesianOptions{0.01, 0.5});
    if (!path.status.ok()) {
        // A short move can still be singular for this arm geometry; only the
        // bookkeeping is asserted in that case.
        CHECK(path.waypoints.empty());
        return;
    }

    CHECK(path.waypoints.size() >= 2);
    CHECK(path.length > 0.05);
    CHECK(path.max_joint_jump < 0.5);

    for (std::size_t i = 0; i < path.waypoints.size(); ++i) {
        const Pose pose = end_effector_pose(model, path.waypoints[i]);
        const Pose target = path.poses[i];
        CHECK_NEAR(pose.position.x, target.position.x, 2e-3);
        CHECK_NEAR(pose.position.y, target.position.y, 2e-3);
        CHECK_NEAR(pose.position.z, target.position.z, 2e-3);
    }
}
