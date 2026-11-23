#pragma once

#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include "core/Math.h"
#include "core/Status.h"
#include "model/ArmModel.h"

namespace af {

enum class ActionType { MoveJoint, MoveLinear, Gripper, Wait, Home };

struct Action {
    ActionType type{ActionType::Wait};
    std::vector<double> joints;  // MoveJoint / Home (empty means all zeros)
    Vec3 target{};               // MoveLinear position (metres, base frame)
    bool close{false};           // Gripper
    double seconds{0.0};         // Wait
};

struct PlanResult {
    bool ok{false};
    std::vector<Action> actions;
    std::string source;  // "rule", "llm" or "fallback"
    Status status;
};

// Narrow command language accepted from an LLM. Keeping the surface this small
// is a safety feature: anything outside the grammar is rejected before it can
// reach the robot.
//
//   move_joint <q1> <q2> <q3> <q4> <q5> <q6>
//   move_linear <x> <y> <z>
//   gripper open|close
//   wait <seconds>
//   home
[[nodiscard]] std::vector<Action> parse_script(std::string_view script);

class ActionPlanner {
public:
    virtual ~ActionPlanner() = default;
    [[nodiscard]] virtual PlanResult plan(std::string_view goal) const = 0;
};

// Keyword driven planner used offline and as the fallback when an LLM answer is
// unusable. It never invents coordinates outside the demo workspace.
class RulePlanner final : public ActionPlanner {
public:
    [[nodiscard]] PlanResult plan(std::string_view goal) const override;
};

class LlmPlanner final : public ActionPlanner {
public:
    using Transport = std::function<std::string(std::string_view prompt)>;

    explicit LlmPlanner(Transport transport, const ActionPlanner* fallback = nullptr);

    [[nodiscard]] PlanResult plan(std::string_view goal) const override;

private:
    Transport transport_;
    const ActionPlanner* fallback_{nullptr};
};

struct ValidationIssue {
    std::size_t action_index{0};
    std::string message;
};

struct ValidationReport {
    bool ok{false};
    std::vector<ValidationIssue> issues;
};

// Checks joint limits, reachability of every Cartesian target and collisions on
// the resulting poses.
[[nodiscard]] ValidationReport validate_actions(const ArmModel& model,
                                                const std::vector<Action>& actions,
                                                const std::vector<double>& start_q);

[[nodiscard]] std::string planner_prompt(std::string_view goal);

}  // namespace af
