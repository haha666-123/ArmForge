#pragma once

#include <string>
#include <vector>

#include "core/Math.h"

namespace af {

struct JointLimit {
    double min{-kPi};
    double max{kPi};
    double max_velocity{1.5};      // rad/s
    double max_acceleration{4.0};  // rad/s^2
};

// Standard (distal) Denavit-Hartenberg row: the transform for joint i is
//   A_i = RotZ(theta_i) * TransZ(d) * TransX(a) * RotX(alpha)
struct DhLink {
    double a{0.0};
    double alpha{0.0};
    double d{0.0};
    double theta_offset{0.0};
};

// A six axis industrial arm. Default parameters follow the UR5 geometry, which
// keeps every number in the README verifiable against public datasheets.
class ArmModel {
public:
    ArmModel() = default;
    ArmModel(std::string name, std::vector<DhLink> links, std::vector<JointLimit> limits,
             std::vector<double> link_radii);

    [[nodiscard]] static ArmModel ur5_like();

    // A collision free, upright parking pose (the all zero pose lies on the floor).
    [[nodiscard]] static std::vector<double> home_configuration();

    [[nodiscard]] const std::string& name() const noexcept { return name_; }
    [[nodiscard]] std::size_t dof() const noexcept { return links_.size(); }
    [[nodiscard]] const std::vector<DhLink>& links() const noexcept { return links_; }
    [[nodiscard]] const std::vector<JointLimit>& limits() const noexcept { return limits_; }
    [[nodiscard]] const std::vector<double>& link_radii() const noexcept { return link_radii_; }

    // Base -> link i+1 transform, with joint value q[i] folded into theta.
    [[nodiscard]] Mat4 joint_transform(std::size_t index, double q) const;

    // World transforms of every link frame, size dof() + 1 (frame 0 is the base).
    [[nodiscard]] std::vector<Mat4> link_frames(const std::vector<double>& q) const;

    [[nodiscard]] Mat4 end_effector(const std::vector<double>& q) const;

    [[nodiscard]] bool within_limits(const std::vector<double>& q) const;
    [[nodiscard]] std::vector<double> clamp_to_limits(const std::vector<double>& q) const;

    // Largest joint limit violation in radians, 0 when inside the limits.
    [[nodiscard]] double limit_violation(const std::vector<double>& q) const;

private:
    std::string name_;
    std::vector<DhLink> links_;
    std::vector<JointLimit> limits_;
    std::vector<double> link_radii_;
};

}  // namespace af
