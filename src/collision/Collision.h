#pragma once

#include <vector>

#include "core/Math.h"
#include "core/Status.h"
#include "model/ArmModel.h"

namespace af {

struct Capsule {
    Vec3 a{};
    Vec3 b{};
    double radius{0.0};
};

// Closest distance between two line segments (Ericson, Real-Time Collision
// Detection, 5.1.9).
[[nodiscard]] double segment_distance(const Vec3& p1, const Vec3& q1, const Vec3& p2,
                                      const Vec3& q2);

[[nodiscard]] std::vector<Capsule> link_capsules(const ArmModel& model,
                                                 const std::vector<double>& q);

struct CollisionReport {
    bool collision{false};
    double min_distance{1e9};
    int link_a{-1};
    int link_b{-1};
    Vec3 closest_point{};
    Status status;
};

// Self collision between non adjacent links plus a ground plane check.
[[nodiscard]] CollisionReport check_collision(const ArmModel& model,
                                              const std::vector<double>& q,
                                              double ground_z = 0.0, double margin = 0.01);

}  // namespace af
