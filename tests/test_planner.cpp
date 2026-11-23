#include "test_framework.h"

#include <stdexcept>

#include "ai/ActionPlanner.h"
#include "model/ArmModel.h"

using namespace af;

TEST(planner_parse_valid_script) {
    const auto actions = parse_script(
        "# pick demo\n"
        "home\n"
        "gripper open\n"
        "move_linear 0.40 0.00 0.30\n"
        "move_joint 0 0 0 0 0 0\n"
        "wait 0.5\n"
        "gripper close\n");

    CHECK(actions.size() == 6);
    CHECK(actions[0].type == ActionType::Home);
    CHECK(actions[1].type == ActionType::Gripper);
    CHECK(!actions[1].close);
    CHECK(actions[2].type == ActionType::MoveLinear);
    CHECK_NEAR(actions[2].target.x, 0.40, 1e-12);
    CHECK(actions[3].type == ActionType::MoveJoint);
    CHECK(actions[3].joints.size() == 6);
    CHECK(actions[4].type == ActionType::Wait);
    CHECK_NEAR(actions[4].seconds, 0.5, 1e-12);
    CHECK(actions[5].close);
}

TEST(planner_rejects_unknown_command) {
    CHECK(parse_script("fly_to_moon 1 2 3\n").empty());
    CHECK(parse_script("move_linear 1 2\n").empty());
    CHECK(parse_script("gripper maybe\n").empty());
}

TEST(planner_rule_pick_and_validate) {
    const ArmModel model = ArmModel::ur5_like();
    const RulePlanner planner;
    const PlanResult plan = planner.plan("抓取零件并抬起来");
    CHECK(plan.ok);
    CHECK(!plan.actions.empty());
    CHECK(plan.source == "rule");

    const ValidationReport report = validate_actions(model, plan.actions,
                                                     ArmModel::home_configuration());
    CHECK(report.ok);
}

TEST(planner_rule_home_and_unknown) {
    const ArmModel model = ArmModel::ur5_like();
    const RulePlanner planner;

    const PlanResult home = planner.plan("回零");
    CHECK(home.ok);
    CHECK(home.actions.size() == 1);

    std::vector<double> q(model.dof(), 0.0);
    const ValidationReport report = validate_actions(model, home.actions, q);
    CHECK(report.ok);

    const PlanResult unknown = planner.plan("帮我写一首诗");
    CHECK(!unknown.ok);
    CHECK(unknown.status.code() == errc::kInvalidArgument);
}

TEST(planner_llm_uses_transport) {
    const ArmModel model = ArmModel::ur5_like();
    auto transport = [](std::string_view prompt) {
        CHECK(prompt.find("move_linear") != std::string_view::npos);
        return std::string("home\ngripper open\n");
    };

    const LlmPlanner planner(transport);
    const PlanResult plan = planner.plan("准备好夹爪");
    CHECK(plan.ok);
    CHECK(plan.source == "llm");
    CHECK(plan.actions.size() == 2);
}

TEST(planner_llm_falls_back_on_garbage) {
    const ArmModel model = ArmModel::ur5_like();
    auto transport = [](std::string_view) { return std::string("I think you should pick it up!\n"); };

    const RulePlanner rule;
    const LlmPlanner planner(transport, &rule);
    const PlanResult plan = planner.plan("抓取零件");

    CHECK(plan.ok);
    CHECK(plan.source == "fallback");
}

TEST(planner_llm_falls_back_on_exception) {
    const ArmModel model = ArmModel::ur5_like();
    auto transport = [](std::string_view) -> std::string { throw std::runtime_error("offline"); };

    const RulePlanner rule;
    const LlmPlanner planner(transport, &rule);
    const PlanResult plan = planner.plan("回零");

    CHECK(plan.ok);
    CHECK(plan.source == "fallback");
}

TEST(planner_validation_rejects_limit_violation) {
    const ArmModel model = ArmModel::ur5_like();
    Action action;
    action.type = ActionType::MoveJoint;
    action.joints = std::vector<double>(model.dof(), 0.0);
    action.joints[0] = model.limits()[0].max + 1.0;

    const ValidationReport report =
        validate_actions(model, {action}, ArmModel::home_configuration());
    CHECK(!report.ok);
    CHECK(!report.issues.empty());
}
