#include "collision/Collision.h"

#include <algorithm>
#include <cmath>

namespace af {

double segment_distance(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2) {
    const Vec3 d1 = q1 - p1;
    const Vec3 d2 = q2 - p2;
    const Vec3 r = p1 - p2;
    const double a = dot(d1, d1);
    const double e = dot(d2, d2);
    const double f = dot(d2, r);

    double s = 0.0;
    double t = 0.0;

    constexpr double kEps = 1e-12;
    if (a <= kEps && e <= kEps) {
        return r.norm();
    }
    if (a <= kEps) {
        s = 0.0;
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        const double c = dot(d1, r);
        if (e <= kEps) {
            t = 0.0;
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            const double b = dot(d1, d2);
            const double denom = a * e - b * b;
            s = denom > kEps ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }

    const Vec3 c1 = p1 + d1 * s;
    const Vec3 c2 = p2 + d2 * t;
    return (c1 - c2).norm();
}

std::vector<Capsule> link_capsules(const ArmModel& model, const std::vector<double>& q) {
    const auto frames = model.link_frames(q);
    std::vector<Capsule> capsules;
    capsules.reserve(model.dof());
    for (std::size_t i = 0; i < model.dof(); ++i) {
        Capsule capsule;
        capsule.a = frames[i].translation();
        capsule.b = frames[i + 1].translation();
        capsule.radius = i < model.link_radii().size() ? model.link_radii()[i] : 0.03;
        capsules.push_back(capsule);
    }
    return capsules;
}

CollisionReport check_collision(const ArmModel& model, const std::vector<double>& q,
                                double ground_z, double margin) {
    CollisionReport report;
    if (q.size() != model.dof()) {
        report.status = Status::error(errc::kInvalidArgument, "joint vector size mismatch");
        return report;
    }

    const auto capsules = link_capsules(model, q);
    for (std::size_t i = 0; i < capsules.size(); ++i) {
        for (std::size_t j = i + 2; j < capsules.size(); ++j) {
            const double distance =
                segment_distance(capsules[i].a, capsules[i].b, capsules[j].a, capsules[j].b);
            const double clearance = distance - capsules[i].radius - capsules[j].radius;
            report.min_distance = std::min(report.min_distance, clearance);
            if (clearance < margin) {
                report.collision = true;
                report.link_a = static_cast<int>(i);
                report.link_b = static_cast<int>(j);
                report.status = Status::error(errc::kCollision,
                                              "links " + std::to_string(i) + " and " +
                                                  std::to_string(j) + " are too close");
                return report;
            }
        }
        // The base column is bolted to the floor, so it is exempt from the ground
        // plane check; every moving link below is not.
        if (i == 0) continue;
        const double ground_clearance =
            std::min(capsules[i].a.z, capsules[i].b.z) - capsules[i].radius - ground_z;
        report.min_distance = std::min(report.min_distance, ground_clearance);
        if (ground_clearance < margin) {
            report.collision = true;
            report.link_a = static_cast<int>(i);
            report.link_b = -1;
            report.status = Status::error(errc::kCollision,
                                          "link " + std::to_string(i) + " hits the ground plane");
            return report;
        }
    }

    report.status = Status::success();
    return report;
}

}  // namespace af
