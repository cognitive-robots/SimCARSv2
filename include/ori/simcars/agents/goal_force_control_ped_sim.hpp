#pragma once

#include <ori/simcars/agents/goal_force_control_ped.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_action_time_conditional.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class GoalForceControlPedSim : public GoalForceControlPed
{
    GoalForceControlPed const* const original_goal_force_control_ped;

protected:
    causal::PedActionTimeConditionalVariable sim_action;

public:
    GoalForceControlPedSim(GoalForceControlPed *goal_force_control_ped, temporal::Time start_time);

    simcars::causal::IEndogenousVariable<PedAction>* get_action_variable() override;
};

}
}
}
