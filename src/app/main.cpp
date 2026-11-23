// ArmForge command line front end.
//
//   armforge demo                     full pipeline, writes out/arm.svg + out/trajectory.csv
//   armforge fk --q "0 0 0 0 0 0"     forward kinematics
//   armforge ik --x 0.4 --y 0.1 --z 0.35
//   armforge plan --goal "抓取零件"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "ai/ActionPlanner.h"
#include "collision/Collision.h"
#include "control/ServoSim.h"
#include "kin/Kinematics.h"
#include "model/ArmModel.h"
#include "traj/Trajectory.h"
#include "viz/SvgRenderer.h"

namespace {

using af::ArmModel;
using af::Pose;
using af::Vec3;

constexpr const char* kVersion = "1.0.0";

void print_pose(const char* label, const Pose& pose) {
    std::cout << label << " position: (" << std::fixed << std::setprecision(4) << pose.position.x
              << ", " << pose.position.y << ", " << pose.position.z << ")\n";
    const auto& r = pose.rotation.m;
    std::cout << "  rotation:\n";
    for (const auto& row : r) {
        std::cout << "    [" << std::setw(8) << row[0] << " " << std::setw(8) << row[1] << " "
                  << std::setw(8) << row[2] << "]\n";
    }
}

std::vector<double> parse_joints(const std::string& text) {
    std::vector<double> values;
    std::stringstream stream(text);
    std::string token;
    while (std::getline(stream, token, ',')) {
        std::stringstream inner(token);
        double value = 0.0;
        while (inner >> value) values.push_back(value);
    }
    return values;
}

double arg_double(const std::map<std::string, std::string>& args, const std::string& key,
                  double fallback) {
    const auto it = args.find(key);
    if (it == args.end()) return fallback;
    try {
        return std::stod(it->second);
    } catch (...) {
        return fallback;
    }
}

int run_demo() {
    const ArmModel model = ArmModel::ur5_like();
    std::cout << "ArmForge " << kVersion << " | " << model.name() << " | dof=" << model.dof()
              << "\n\n";

    const std::vector<double> home = ArmModel::home_configuration();
    const Pose home_pose = af::end_effector_pose(model, home);
    print_pose("home", home_pose);

    // 1. Inverse kinematics: take a reachable joint configuration, compute its
    //    Cartesian pose, then ask the solver to find it again from the home seed.
    const std::vector<double> goal_q{0.35, -0.90, 1.15, 0.30, 0.75, 0.20};
    Pose target = af::end_effector_pose(model, goal_q);
    std::cout << "\ntarget taken from q = [";
    for (std::size_t i = 0; i < goal_q.size(); ++i) std::cout << (i ? " " : "") << goal_q[i];
    std::cout << "]\n";
    print_pose("target", target);

    const auto ik = af::solve_ik(model, target, home);
    std::cout << "\nik: " << (ik.ok ? "converged" : "FAILED") << " in " << ik.iterations
              << " iters, pos_err=" << std::scientific << ik.position_error
              << " rot_err=" << ik.orientation_error << std::fixed << "\n";
    if (!ik.ok) return 1;
    print_pose("ik", af::end_effector_pose(model, ik.q));

    // 2. Joint space trajectory home -> ik.q
    const auto trajectory = af::JointTrajectory::build(model, home, ik.q);
    const auto stats = trajectory.stats();
    std::cout << "\ntrajectory: duration=" << std::setprecision(3) << trajectory.duration()
              << " s, peak joint vel=" << stats.max_velocity
              << " rad/s, peak acc=" << stats.max_acceleration << " rad/s^2\n";

    // 3. Servo tracking simulation at 100 Hz
    const auto servo = af::simulate_servo(model, trajectory);
    std::cout << "servo sim: " << servo.steps.size() << " cycles, max err=" << std::scientific
              << servo.max_error << " rad, rms err=" << servo.rms_error << std::fixed
              << ", limits respected=" << (servo.within_limits ? "yes" : "no") << "\n";

    // 4. Collision check on the goal configuration
    const auto collision = af::check_collision(model, ik.q);
    std::cout << "collision: " << (collision.collision ? "DETECTED" : "clear") << ", min clearance="
              << std::setprecision(4) << collision.min_distance << " m\n";

    // 5. Natural language task -> validated action sequence
    const af::RulePlanner planner;
    const auto plan = planner.plan("抓取零件并抬起来");
    const auto validation = af::validate_actions(model, plan.actions, home);
    std::cout << "planner: " << plan.actions.size() << " actions from \"" << plan.source
              << "\", validation=" << (validation.ok ? "ok" : "failed") << "\n";
    for (const auto& issue : validation.issues) {
        std::cout << "  issue at action " << issue.action_index << ": " << issue.message << "\n";
    }

    // 6. Render an SVG with sampled poses and the end effector path
    std::filesystem::create_directories("out");
    std::vector<std::vector<double>> sampled_poses;
    std::vector<Vec3> ee_path;
    const double dt = trajectory.duration() / 40.0;
    for (int i = 0; i <= 40; ++i) {
        const double t = std::min(trajectory.duration(), dt * i);
        const auto q = trajectory.sample(t);
        sampled_poses.push_back(q);
        ee_path.push_back(af::end_effector_pose(model, q).position);
    }

    std::ofstream svg_file("out/arm.svg");
    svg_file << af::render_svg(model, sampled_poses, ee_path);

    std::ofstream csv("out/trajectory.csv");
    csv << "t";
    for (std::size_t i = 0; i < model.dof(); ++i) csv << ",q" << (i + 1);
    csv << "\n";
    for (int i = 0; i <= 200; ++i) {
        const double t = trajectory.duration() * static_cast<double>(i) / 200.0;
        const auto q = trajectory.sample(t);
        csv << t;
        for (double value : q) csv << "," << value;
        csv << "\n";
    }

    std::cout << "\nwrote out/arm.svg and out/trajectory.csv\n";
    return 0;
}

int run_fk(const std::map<std::string, std::string>& args) {
    const ArmModel model = ArmModel::ur5_like();
    const auto it = args.find("q");
    if (it == args.end()) {
        std::cerr << "fk needs --q \"q1 q2 q3 q4 q5 q6\"\n";
        return 2;
    }
    const auto q = parse_joints(it->second);
    if (q.size() != model.dof()) {
        std::cerr << "expected " << model.dof() << " joint values, got " << q.size() << "\n";
        return 2;
    }
    print_pose("fk", af::end_effector_pose(model, q));
    const auto collision = af::check_collision(model, q);
    std::cout << "collision: " << (collision.collision ? "DETECTED" : "clear")
              << ", min clearance=" << collision.min_distance << " m\n";
    return 0;
}

int run_ik(const std::map<std::string, std::string>& args) {
    const ArmModel model = ArmModel::ur5_like();
    Pose target;
    target.position = Vec3{arg_double(args, "x", 0.4), arg_double(args, "y", 0.0),
                           arg_double(args, "z", 0.3)};
    target.rotation = af::end_effector_pose(model, std::vector<double>(model.dof(), 0.0)).rotation;

    const auto seed_it = args.find("seed");
    const auto seed = seed_it == args.end() ? ArmModel::home_configuration()
                                            : parse_joints(seed_it->second);
    af::IkOptions options;
    options.position_only = args.find("pos-only") != args.end();
    const auto ik = af::solve_ik(model, target, seed, options);
    std::cout << (ik.ok ? "converged" : "failed") << " in " << ik.iterations
              << " iters, pos_err=" << ik.position_error << ", rot_err=" << ik.orientation_error
              << "\nq:";
    for (double value : ik.q) std::cout << " " << value;
    std::cout << "\n";
    return ik.ok ? 0 : 1;
}

int run_plan(const std::map<std::string, std::string>& args) {
    const ArmModel model = ArmModel::ur5_like();
    const auto it = args.find("goal");
    const std::string goal = it == args.end() ? "抓取零件" : it->second;

    const af::RulePlanner planner;
    const auto plan = planner.plan(goal);
    if (!plan.ok) {
        std::cerr << "cannot plan: " << plan.status.message() << "\n";
        return 1;
    }
    const auto validation = af::validate_actions(model, plan.actions, std::vector<double>(model.dof(), 0.0));
    std::cout << "goal: " << goal << "\nactions: " << plan.actions.size()
              << ", source: " << plan.source << ", validation: " << (validation.ok ? "ok" : "failed")
              << "\n";
    for (std::size_t i = 0; i < plan.actions.size(); ++i) {
        const auto& action = plan.actions[i];
        std::cout << "  " << i + 1 << ". ";
        switch (action.type) {
            case af::ActionType::MoveJoint: std::cout << "move_joint"; break;
            case af::ActionType::MoveLinear:
                std::cout << "move_linear (" << action.target.x << ", " << action.target.y << ", "
                          << action.target.z << ")";
                break;
            case af::ActionType::Gripper: std::cout << (action.close ? "gripper close" : "gripper open"); break;
            case af::ActionType::Wait: std::cout << "wait " << action.seconds; break;
            case af::ActionType::Home: std::cout << "home"; break;
        }
        std::cout << "\n";
    }
    for (const auto& issue : validation.issues) {
        std::cout << "  issue at action " << issue.action_index << ": " << issue.message << "\n";
    }
    return validation.ok ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string command = argc > 1 ? argv[1] : "demo";

    std::map<std::string, std::string> args;
    for (int i = 2; i < argc; ++i) {
        std::string key = argv[i];
        if (key.rfind("--", 0) != 0) continue;
        key = key.substr(2);
        if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
            args[key] = argv[++i];
        } else {
            args[key] = "1";
        }
    }

    if (command == "--help" || command == "-h" || command == "help") {
        std::cout << "ArmForge " << kVersion << "\n"
                  << "  armforge demo\n"
                  << "  armforge fk --q \"0 0 0 0 0 0\"\n"
                  << "  armforge ik --x 0.4 --y 0.1 --z 0.35\n"
                  << "  armforge plan --goal \"抓取零件\"\n";
        return 0;
    }
    if (command == "fk") return run_fk(args);
    if (command == "ik") return run_ik(args);
    if (command == "plan") return run_plan(args);
    return run_demo();
}
