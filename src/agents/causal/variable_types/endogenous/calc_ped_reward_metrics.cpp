
#include <ori/simcars/agents/causal/variable_types/endogenous/calc_ped_reward_metrics.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

CalcPedRewardMetricsVariable::CalcPedRewardMetricsVariable(
        simcars::causal::IEndogenousVariable<PedOutcomeActionPair> *endogenous_parent_1,
        simcars::causal::IEndogenousVariable<PedTask> *endogenous_parent_2,
        simcars::causal::IVariable<PedRewardParameters> *other_parent,
        IPedRewardCalc const *ped_reward_calculator) :
    ATernaryEndogenousVariable(endogenous_parent_1, endogenous_parent_2, other_parent),
    ped_reward_calculator(ped_reward_calculator) {}

bool CalcPedRewardMetricsVariable::get_value(PedRewards &val) const
{
    PedOutcomeActionPair outcome_action_pair;
    PedTask task;
    PedRewardParameters reward_parameters;
    if (get_endogenous_parent_1()->get_value(outcome_action_pair) &&
            get_endogenous_parent_2()->get_value(task) &&
            get_other_parent()->get_value(reward_parameters))
    {
        PedRewards reward_metrics = ped_reward_calculator->calc_rewards(
                    &(outcome_action_pair.first), &task, &reward_parameters);
        val = reward_metrics;

        return true;
    }
    else
    {
        return false;
    }
}

bool CalcPedRewardMetricsVariable::set_value(PedRewards const &val)
{
    PedOutcomeActionPair outcome_action_pair;
    PedTask task;
    PedRewardParameters reward_parameters;
    if (get_endogenous_parent_1()->get_value(outcome_action_pair) &&
            get_endogenous_parent_2()->get_value(task) &&
            get_other_parent()->get_value(reward_parameters))
    {
        PedRewards reward_metrics = ped_reward_calculator->calc_rewards(
                    &(outcome_action_pair.first), &task, &reward_parameters);

        return val == reward_metrics;
    }
    else
    {
        return true;
    }
}

}
}
}
}
