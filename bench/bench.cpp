// Micro benchmarks: how fast are FK, the Jacobian, IK and trajectory sampling?
// Numbers are printed so the README can quote the measured values.

#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

#include "kin/Kinematics.h"
#include "model/ArmModel.h"
#include "traj/Trajectory.h"

namespace {

using Clock = std::chrono::steady_clock;

double seconds_since(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

}  // namespace

int main() {
    const af::ArmModel model = af::ArmModel::ur5_like();
    const auto home = af::ArmModel::home_configuration();
    std::vector<double> goal = home;
    goal[0] += 0.6;
    goal[1] -= 0.4;
    goal[2] += 0.2;

    constexpr int kIterations = 20000;

    // Forward kinematics + Jacobian
    {
        const auto start = Clock::now();
        double sink = 0.0;
        for (int i = 0; i < kIterations; ++i) {
            sink += af::end_effector_pose(model, goal).position.x;
            sink += af::geometric_jacobian(model, goal)(0, 0);
        }
        const double elapsed = seconds_since(start);
        std::cout << "fk+jacobian : " << std::fixed << std::setprecision(1)
                  << kIterations / elapsed << " calls/s (" << elapsed / kIterations * 1e6
                  << " us each) [sink=" << sink << "]\n";
    }

    // Inverse kinematics: targets sampled across the joint range, solved from a
    // single seed (home), so the convergence rate is realistic rather than tuned.
    {
        const int solves = 2000;
        const auto start = Clock::now();
        int converged = 0;
        unsigned int state = 12345u;
        auto next_random = [&state]() {
            state = state * 1664525u + 1013904223u;
            return static_cast<double>(state % 10000u) / 10000.0;
        };
        for (int i = 0; i < solves; ++i) {
            std::vector<double> q(model.dof(), 0.0);
            for (std::size_t j = 0; j < q.size(); ++j) q[j] = (next_random() * 2.4 - 1.2);
            const af::Pose target = af::end_effector_pose(model, q);
            const auto result = af::solve_ik(model, target, home);
            if (result.ok) ++converged;
        }
        const double elapsed = seconds_since(start);
        std::cout << "ik          : " << solves / elapsed << " solves/s (" << elapsed / solves * 1e3
                  << " ms each), converged " << converged << "/" << solves << "\n";
    }

    // Same targets through the multi-start wrapper: the extra seeds trade time
    // for a much better success rate on awkward targets.
    {
        const int solves = 2000;
        const auto start = Clock::now();
        int converged = 0;
        unsigned int state = 12345u;
        auto next_random = [&state]() {
            state = state * 1664525u + 1013904223u;
            return static_cast<double>(state % 10000u) / 10000.0;
        };
        for (int i = 0; i < solves; ++i) {
            std::vector<double> q(model.dof(), 0.0);
            for (std::size_t j = 0; j < q.size(); ++j) q[j] = (next_random() * 2.4 - 1.2);
            const af::Pose target = af::end_effector_pose(model, q);
            const auto result = af::solve_ik_best(model, target, home);
            if (result.ok) ++converged;
        }
        const double elapsed = seconds_since(start);
        std::cout << "ik (multi)  : " << solves / elapsed << " solves/s (" << elapsed / solves * 1e3
                  << " ms each), converged " << converged << "/" << solves << "\n";
    }

    // Trajectory build + sampling
    {
        const af::JointTrajectory trajectory = af::JointTrajectory::build(model, home, goal);
        const int samples = 200000;
        const auto start = Clock::now();
        double sink = 0.0;
        for (int i = 0; i < samples; ++i) {
            const double t = trajectory.duration() * static_cast<double>(i % 1000) / 999.0;
            sink += trajectory.sample(t)[0];
        }
        const double elapsed = seconds_since(start);
        std::cout << "trajectory  : " << samples / elapsed << " samples/s (" << elapsed / samples * 1e6
                  << " us each) [sink=" << sink << "]\n";
    }

    return 0;
}
