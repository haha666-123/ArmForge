#include "test_framework.h"

#include "kin/Kinematics.h"
#include "model/ArmModel.h"

using namespace af;

TEST(model_shape) {
    const ArmModel model = ArmModel::ur5_like();
    CHECK(model.dof() == 6);
    CHECK(model.links().size() == 6);
    CHECK(model.limits().size() == 6);
    CHECK(model.link_radii().size() == 6);
}

TEST(model_home_is_valid_and_clear_of_the_floor) {
    const ArmModel model = ArmModel::ur5_like();
    const auto home = ArmModel::home_configuration();
    CHECK(home.size() == model.dof());
    CHECK(model.within_limits(home));

    const Pose pose = end_effector_pose(model, home);
    CHECK(pose.position.z > 0.1);
    CHECK(pose.position.norm() < 1.0);
}

TEST(model_clamp_to_limits) {
    const ArmModel model = ArmModel::ur5_like();
    std::vector<double> q(6, 0.0);
    q[0] = 10.0;
    q[1] = -10.0;

    const auto clamped = model.clamp_to_limits(q);
    CHECK(clamped[0] <= model.limits()[0].max + 1e-12);
    CHECK(clamped[1] >= model.limits()[1].min - 1e-12);
    CHECK(model.within_limits(clamped));
}

TEST(model_limit_violation) {
    const ArmModel model = ArmModel::ur5_like();
    std::vector<double> q(6, 0.0);
    CHECK_NEAR(model.limit_violation(q), 0.0, 1e-12);

    q[2] = model.limits()[2].max + 0.5;
    CHECK(model.limit_violation(q) > 0.4);
}

TEST(model_link_frames) {
    const ArmModel model = ArmModel::ur5_like();
    const auto frames = model.link_frames(std::vector<double>(6, 0.0));
    CHECK(frames.size() == 7);
    CHECK_NEAR(frames.front().translation().norm(), 0.0, 1e-12);
    CHECK(frames.back().translation().norm() > 0.5);
}
