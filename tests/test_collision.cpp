#include "test_framework.h"

#include "collision/Collision.h"
#include "model/ArmModel.h"

using namespace af;

TEST(collision_segment_distance_parallel) {
    const double distance = segment_distance(Vec3{0, 0, 0}, Vec3{1, 0, 0}, Vec3{0, 1, 0},
                                             Vec3{1, 1, 0});
    CHECK_NEAR(distance, 1.0, 1e-12);
}

TEST(collision_segment_distance_crossing) {
    const double distance = segment_distance(Vec3{-1, 0, 0}, Vec3{1, 0, 0}, Vec3{0, -1, 0},
                                             Vec3{0, 1, 0});
    CHECK_NEAR(distance, 0.0, 1e-12);
}

TEST(collision_segment_distance_degenerate_point) {
    const double distance = segment_distance(Vec3{0, 0, 0}, Vec3{0, 0, 0}, Vec3{3, 0, 0},
                                             Vec3{3, 0, 0});
    CHECK_NEAR(distance, 3.0, 1e-12);
}

TEST(collision_home_pose_is_clear) {
    const ArmModel model = ArmModel::ur5_like();
    const CollisionReport report = check_collision(model, ArmModel::home_configuration());
    CHECK(!report.collision);
    CHECK(report.min_distance > 0.0);
    CHECK(report.status.ok());
}

TEST(collision_ground_plane_is_detected) {
    const ArmModel model = ArmModel::ur5_like();
    // Raising the floor above the parked arm must trigger the ground check.
    const CollisionReport report = check_collision(model, ArmModel::home_configuration(), 0.5, 0.01);
    CHECK(report.collision);
    CHECK(report.status.code() == errc::kCollision);
}

TEST(collision_capsule_count) {
    const ArmModel model = ArmModel::ur5_like();
    const auto capsules = link_capsules(model, ArmModel::home_configuration());
    CHECK(capsules.size() == model.dof());
    for (const auto& capsule : capsules) CHECK(capsule.radius > 0.0);
}
