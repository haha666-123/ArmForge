#include "model/ArmModel.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace af {

ArmModel::ArmModel(std::string name, std::vector<DhLink> links, std::vector<JointLimit> limits,
                   std::vector<double> link_radii)
    : name_(std::move(name)),
      links_(std::move(links)),
      limits_(std::move(limits)),
      link_radii_(std::move(link_radii)) {}

ArmModel ArmModel::ur5_like() {
    // UR5 datasheet: a2 = -0.425, a3 = -0.39225, d1 = 0.089159, d4 = 0.10915,
    // d5 = 0.09465, d6 = 0.0823 (metres).
    std::vector<DhLink> links{
        {0.0, kPi / 2.0, 0.089159, 0.0},
        {-0.425, 0.0, 0.0, 0.0},
        {-0.39225, 0.0, 0.0, 0.0},
        {0.0, kPi / 2.0, 0.10915, 0.0},
        {0.0, -kPi / 2.0, 0.09465, 0.0},
        {0.0, 0.0, 0.0823, 0.0},
    };

    // Conservative working range: +/-170 degrees, except the elbow which folds
    // inward on the UR5.
    std::vector<JointLimit> limits{
        {-170.0 * kDegToRad, 170.0 * kDegToRad, 2.0, 6.0},
        {-170.0 * kDegToRad, 170.0 * kDegToRad, 2.0, 6.0},
        {-170.0 * kDegToRad, 170.0 * kDegToRad, 2.0, 6.0},
        {-170.0 * kDegToRad, 170.0 * kDegToRad, 3.0, 10.0},
        {-170.0 * kDegToRad, 170.0 * kDegToRad, 3.0, 10.0},
        {-170.0 * kDegToRad, 170.0 * kDegToRad, 3.0, 10.0},
    };

    // Link radii used by the collision model (upper arm and forearm are the fat
    // parts, the wrist links are thin).
    std::vector<double> radii{0.06, 0.05, 0.05, 0.035, 0.03, 0.03};

    return ArmModel("ur5-like", std::move(links), std::move(limits), std::move(radii));
}

std::vector<double> ArmModel::home_configuration() {
    return {0.0, -1.2, 1.2, 0.0, 0.9, 0.0};
}

Mat4 ArmModel::joint_transform(std::size_t index, double q) const {
    const DhLink& link = links_.at(index);
    const double theta = q + link.theta_offset;

    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    const double ca = std::cos(link.alpha);
    const double sa = std::sin(link.alpha);

    Mat4 out = Mat4::identity();
    out.m[0][0] = ct;
    out.m[0][1] = -st * ca;
    out.m[0][2] = st * sa;
    out.m[0][3] = link.a * ct;
    out.m[1][0] = st;
    out.m[1][1] = ct * ca;
    out.m[1][2] = -ct * sa;
    out.m[1][3] = link.a * st;
    out.m[2][0] = 0.0;
    out.m[2][1] = sa;
    out.m[2][2] = ca;
    out.m[2][3] = link.d;
    return out;
}

std::vector<Mat4> ArmModel::link_frames(const std::vector<double>& q) const {
    std::vector<Mat4> frames;
    frames.reserve(links_.size() + 1);
    frames.push_back(Mat4::identity());
    Mat4 current = Mat4::identity();
    for (std::size_t i = 0; i < links_.size(); ++i) {
        current = current * joint_transform(i, i < q.size() ? q[i] : 0.0);
        frames.push_back(current);
    }
    return frames;
}

Mat4 ArmModel::end_effector(const std::vector<double>& q) const {
    const auto frames = link_frames(q);
    return frames.back();
}

bool ArmModel::within_limits(const std::vector<double>& q) const {
    if (q.size() != links_.size()) return false;
    for (std::size_t i = 0; i < q.size(); ++i) {
        if (q[i] < limits_[i].min - 1e-9 || q[i] > limits_[i].max + 1e-9) return false;
    }
    return true;
}

std::vector<double> ArmModel::clamp_to_limits(const std::vector<double>& q) const {
    std::vector<double> out = q;
    const std::size_t n = std::min(out.size(), limits_.size());
    for (std::size_t i = 0; i < n; ++i) {
        out[i] = std::clamp(out[i], limits_[i].min, limits_[i].max);
    }
    return out;
}

double ArmModel::limit_violation(const std::vector<double>& q) const {
    double worst = 0.0;
    const std::size_t n = std::min(q.size(), limits_.size());
    for (std::size_t i = 0; i < n; ++i) {
        if (q[i] < limits_[i].min) worst = std::max(worst, limits_[i].min - q[i]);
        if (q[i] > limits_[i].max) worst = std::max(worst, q[i] - limits_[i].max);
    }
    return worst;
}

}  // namespace af
