#include "test_framework.h"

#include <vector>

#include "kin/Kinematics.h"
#include "model/ArmModel.h"

using namespace af;

TEST(kinematics_fk_is_finite) {
    const ArmModel model = ArmModel::ur5_like();
    const Pose pose = end_effector_pose(model, ArmModel::home_configuration());
    CHECK(std::isfinite(pose.position.x));
    CHECK(std::isfinite(pose.position.y));
    CHECK(std::isfinite(pose.position.z));

    const Mat3 r = pose.rotation;
    const Mat3 identity = r.transposed() * r;
    CHECK_NEAR(identity.m[0][0], 1.0, 1e-9);
    CHECK_NEAR(identity.m[1][1], 1.0, 1e-9);
    CHECK_NEAR(identity.m[2][2], 1.0, 1e-9);
}

TEST(kinematics_ik_round_trip) {
    const ArmModel model = ArmModel::ur5_like();
    const std::vector<double> goal{0.35, -0.90, 1.15, 0.30, 0.75, 0.20};
    const Pose target = end_effector_pose(model, goal);

    const IkResult result = solve_ik(model, target, ArmModel::home_configuration());
    CHECK(result.ok);
    CHECK(result.position_error < 1e-4);
    CHECK(result.orientation_error < 1e-3);
    CHECK(result.iterations < 200);
}

TEST(kinematics_ik_rejects_unreachable_target) {
    const ArmModel model = ArmModel::ur5_like();
    Pose target;
    target.position = Vec3{5.0, 5.0, 5.0};
    target.rotation = Mat3::identity();

    const IkResult result = solve_ik(model, target, ArmModel::home_configuration());
    CHECK(!result.ok);
    CHECK(result.status.code() == errc::kNoConvergence);
}

TEST(kinematics_jacobian_matches_finite_difference) {
    const ArmModel model = ArmModel::ur5_like();
    const std::vector<double> q{0.2, -0.7, 0.9, 0.1, 0.6, -0.2};
    const DenseMatrix jacobian = geometric_jacobian(model, q);

    const double epsilon = 1e-6;
    for (std::size_t i = 0; i < model.dof(); ++i) {
        std::vector<double> perturbed = q;
        perturbed[i] += epsilon;
        const Vec3 base = end_effector_pose(model, q).position;
        const Vec3 moved = end_effector_pose(model, perturbed).position;
        const Vec3 derivative = (moved - base) / epsilon;
        CHECK_NEAR(jacobian(0, i), derivative.x, 1e-4);
        CHECK_NEAR(jacobian(1, i), derivative.y, 1e-4);
        CHECK_NEAR(jacobian(2, i), derivative.z, 1e-4);
    }
}

TEST(kinematics_multi_start_finds_solution) {
    const ArmModel model = ArmModel::ur5_like();
    Pose target;
    target.position = Vec3{0.40, 0.0, 0.30};
    target.rotation = Mat3::identity();

    IkOptions options;
    options.position_only = true;
    const IkResult result = solve_ik_best(model, target, ArmModel::home_configuration(), options);
    CHECK(result.ok);
    CHECK(result.position_error < 1e-3);
}

TEST(kinematics_position_only_ik) {
    const ArmModel model = ArmModel::ur5_like();
    const Pose target = end_effector_pose(model, std::vector<double>{0.6, -1.1, 1.0, 0.2, 0.4, 0.1});

    IkOptions options;
    options.position_only = true;
    const IkResult result = solve_ik(model, target, ArmModel::home_configuration(), options);
    CHECK(result.ok);
    CHECK(result.position_error < 1e-4);
}
