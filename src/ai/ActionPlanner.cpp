#include "ai/ActionPlanner.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include "collision/Collision.h"
#include "kin/Kinematics.h"

namespace af {

namespace {

std::vector<double> parse_numbers(std::istringstream& stream) {
    std::vector<double> values;
    double value = 0.0;
    while (stream >> value) values.push_back(value);
    return values;
}

bool contains_any(std::string_view text, std::initializer_list<std::string_view> needles) {
    for (const auto needle : needles) {
        if (text.find(needle) != std::string_view::npos) return true;
    }
    return false;
}

}  // namespace

std::vector<Action> parse_script(std::string_view script) {
    std::vector<Action> actions;
    std::istringstream lines{std::string(script)};
    std::string line;

    while (std::getline(lines, line)) {
        std::istringstream stream(line);
        std::string command;
        if (!(stream >> command)) continue;  // blank line
        if (command.empty() || command[0] == '#') continue;

        if (command == "move_joint") {
            Action action;
            action.type = ActionType::MoveJoint;
            action.joints = parse_numbers(stream);
            actions.push_back(std::move(action));
        } else if (command == "move_linear") {
            Action action;
            action.type = ActionType::MoveLinear;
            const auto values = parse_numbers(stream);
            if (values.size() != 3) return {};
            action.target = Vec3{values[0], values[1], values[2]};
            actions.push_back(std::move(action));
        } else if (command == "gripper") {
            std::string state;
            if (!(stream >> state)) return {};
            if (state != "open" && state != "close") return {};
            Action action;
            action.type = ActionType::Gripper;
            action.close = state == "close";
            actions.push_back(std::move(action));
        } else if (command == "wait") {
            double seconds = 0.0;
            if (!(stream >> seconds) || seconds < 0.0) return {};
            Action action;
            action.type = ActionType::Wait;
            action.seconds = seconds;
            actions.push_back(std::move(action));
        } else if (command == "home") {
            Action action;
            action.type = ActionType::Home;
            actions.push_back(std::move(action));
        } else {
            return {};  // unknown command: reject the whole script
        }
    }
    return actions;
}

PlanResult RulePlanner::plan(std::string_view goal) const {
    PlanResult result;
    result.source = "rule";

    if (contains_any(goal, {"抓", "pick", "pickup", "拿"})) {
        result.actions = parse_script(
            "home\n"
            "gripper open\n"
            "move_linear 0.40 0.00 0.30\n"
            "gripper close\n"
            "move_linear 0.40 0.00 0.45\n");
        result.ok = true;
        result.status = Status::success();
        return result;
    }

    if (contains_any(goal, {"回零", "原点", "home", "复位"})) {
        result.actions = parse_script("home\n");
        result.ok = true;
        result.status = Status::success();
        return result;
    }

    if (contains_any(goal, {"放下", "place", "释放"})) {
        result.actions = parse_script(
            "move_linear 0.45 0.15 0.30\n"
            "gripper open\n"
            "move_linear 0.45 0.15 0.45\n");
        result.ok = true;
        result.status = Status::success();
        return result;
    }

    result.ok = false;
    result.status = Status::error(errc::kInvalidArgument,
                                  "无法理解的指令，可试试：抓取零件 / 放下 / 回零");
    return result;
}

LlmPlanner::LlmPlanner(Transport transport, const ActionPlanner* fallback)
    : transport_(std::move(transport)), fallback_(fallback) {}

std::string planner_prompt(std::string_view goal) {
    std::ostringstream out;
    out << "You are a robot task planner. Convert the request into the command language below.\n"
        << "Answer with commands only, one per line, no prose and no code fences.\n\n"
        << "move_joint <q1> <q2> <q3> <q4> <q5> <q6>   (radians)\n"
        << "move_linear <x> <y> <z>                    (metres, base frame)\n"
        << "gripper open|close\n"
        << "wait <seconds>\n"
        << "home\n\n"
        << "Request: " << goal << "\n";
    return out.str();
}

PlanResult LlmPlanner::plan(std::string_view goal) const {
    PlanResult result;
    result.source = "llm";

    if (!transport_) {
        result.status = Status::error(errc::kInternal, "no LLM transport configured");
        return result;
    }

    std::string script;
    try {
        script = transport_(planner_prompt(goal));
    } catch (const std::exception& error) {
        result.status = Status::error(errc::kInternal,
                                      std::string("LLM transport failed: ") + error.what());
        if (fallback_ != nullptr) {
            auto fallback = fallback_->plan(goal);
            fallback.source = "fallback";
            return fallback;
        }
        return result;
    }

    // Tolerate a fenced block even though the prompt forbids it.
    const auto fence_begin = script.find("```");
    if (fence_begin != std::string::npos) {
        const auto newline = script.find('\n', fence_begin);
        const auto fence_end = script.find("```", newline == std::string::npos ? fence_begin + 3 : newline);
        if (newline != std::string::npos && fence_end != std::string::npos) {
            script = script.substr(newline + 1, fence_end - newline - 1);
        }
    }

    const auto actions = parse_script(script);
    if (actions.empty()) {
        result.status = Status::error(errc::kParseError, "LLM answer did not match the command grammar");
        if (fallback_ != nullptr) {
            auto fallback = fallback_->plan(goal);
            fallback.source = "fallback";
            return fallback;
        }
        return result;
    }

    result.actions = actions;
    result.ok = true;
    result.status = Status::success();
    return result;
}

ValidationReport validate_actions(const ArmModel& model, const std::vector<Action>& actions,
                                  const std::vector<double>& start_q) {
    ValidationReport report;
    if (actions.empty()) {
        report.issues.push_back({0, "plan is empty"});
        return report;
    }
    if (start_q.size() != model.dof()) {
        report.issues.push_back({0, "start configuration size mismatch"});
        return report;
    }

    std::vector<double> q = model.clamp_to_limits(start_q);

    for (std::size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];
        switch (action.type) {
            case ActionType::MoveJoint: {
                if (action.joints.size() != model.dof()) {
                    report.issues.push_back({i, "move_joint needs " + std::to_string(model.dof()) +
                                                    " values"});
                    break;
                }
                if (!model.within_limits(action.joints)) {
                    report.issues.push_back({i, "move_joint exceeds joint limits"});
                    break;
                }
                q = action.joints;
                break;
            }
            case ActionType::MoveLinear: {
                Pose target;
                target.position = action.target;
                target.rotation = end_effector_pose(model, q).rotation;
                IkOptions options;
                options.max_iterations = 150;
                // Pick and place only constrains the tool centre point; the wrist
                // orientation is free to follow whatever the arm can reach.
                options.position_only = true;
                const IkResult ik = solve_ik_best(model, target, q, options);
                if (!ik.ok) {
                    report.issues.push_back(
                        {i, "move_linear target is unreachable: " + ik.status.message()});
                    break;
                }
                q = ik.q;
                break;
            }
            case ActionType::Home: {
                q = ArmModel::home_configuration();
                break;
            }
            case ActionType::Gripper:
            case ActionType::Wait: break;
        }

        const CollisionReport collision = check_collision(model, q);
        if (collision.collision) {
            report.issues.push_back({i, "collision: " + collision.status.message()});
        }
    }

    report.ok = report.issues.empty();
    return report;
}

}  // namespace af
