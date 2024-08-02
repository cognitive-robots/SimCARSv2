
#include <ori/simcars/agents/causal/variable_types/endogenous/calc_ped_action_outcome_reward.hpp>

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

CalcPedActionOutcomeRewardVariable::CalcPedActionOutcomeRewardVariable(
        simcars::causal::IEndogenousVariable<PedOutcomeActionPairs> *endogenous_parent,
        simcars::causal::IVariable<PedRewardParameters> *other_parent,
        IPedRewardCalc const *ped_reward_calculator) :
    ABinaryEndogenousVariable(endogenous_parent, other_parent),
    ped_reward_calculator(ped_reward_calculator) {}

bool CalcPedActionOutcomeRewardVariable::get_value(RewardPedOutcomeActionTuples &val) const
{
    PedOutcomeActionPairs outcome_action_pairs;
    PedRewardParameters reward_parameters;
    if (get_endogenous_parent()->get_value(outcome_action_pairs) && get_other_parent()->get_value(reward_parameters))
    {
        val.clear();

        for (size_t i = 0; i < outcome_action_pairs.count(); ++i)
        {
            FP_DATA_TYPE reward = ped_reward_calculator->calc_reward(
                        &(outcome_action_pairs[i].first),
                        &reward_parameters);
            val.push_back(RewardPedOutcomeActionTuple(reward, outcome_action_pairs[i].first,
                                                      outcome_action_pairs[i].second));
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool CalcPedActionOutcomeRewardVariable::set_value(RewardPedOutcomeActionTuples const &val)
{
    // WARNING: Assumes action ordering is the same in both arrays
    // TODO: Potentially consider separating reward weightings from other reward parameters
    PedRewardParameters reward_parameters;
    if (get_other_parent()->get_value(reward_parameters))
    {
        Eigen::MatrixXd individual_rewards(val.count(), 1);
        Eigen::VectorXd combined_rewards(val.count());
        for (size_t i = 0; i < val.count(); ++i)
        {
            PedRewards rewards = ped_reward_calculator->calc_rewards(
                        &(std::get<1>(val[i])), &reward_parameters);
            individual_rewards(i, 0) = 1.0;
            combined_rewards(i) = std::get<0>(val[i]);
        }
        Eigen::VectorXd reward_weights =
                individual_rewards.colPivHouseholderQr().solve(combined_rewards);
        reward_parameters.bias_weight = reward_weights(0);

        return get_other_parent()->set_value(reward_parameters);
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
