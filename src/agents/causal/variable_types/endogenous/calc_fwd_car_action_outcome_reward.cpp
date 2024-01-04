
#include <ori/simcars/agents/causal/variable_types/endogenous/calc_fwd_car_action_outcome_reward.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

CalcFWDCarActionOutcomeRewardVariable::CalcFWDCarActionOutcomeRewardVariable(
        simcars::causal::IEndogenousVariable<structures::stl::STLStackArray<FWDCarOutcomeActionPair>> *endogenous_parent,
        simcars::causal::IVariable<FWDCarRewardParameters> *other_parent,
        IFWDCarRewardCalc const *fwd_car_reward_calculator) :
    ABinaryEndogenousVariable(endogenous_parent, other_parent),
    fwd_car_reward_calculator(fwd_car_reward_calculator) {}

bool CalcFWDCarActionOutcomeRewardVariable::get_value(structures::stl::STLStackArray<RewardFWDCarActionPair> &val) const
{
    structures::stl::STLStackArray<FWDCarOutcomeActionPair> outcome_action_pairs;
    FWDCarRewardParameters reward_parameters;
    if (get_endogenous_parent()->get_value(outcome_action_pairs) && get_other_parent()->get_value(reward_parameters))
    {
        val.clear();

        for (size_t i = 0; i < outcome_action_pairs.count(); ++i)
        {
            val.push_back(RewardFWDCarActionPair(fwd_car_reward_calculator->calc_reward(
                                                     &(outcome_action_pairs[i].first),
                                                     &reward_parameters),
                                                 outcome_action_pairs[i].second));
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool CalcFWDCarActionOutcomeRewardVariable::set_value(structures::stl::STLStackArray<RewardFWDCarActionPair> const &val)
{
    structures::stl::STLStackArray<FWDCarOutcomeActionPair> outcome_action_pairs;
    FWDCarRewardParameters reward_parameters;
    if (get_endogenous_parent()->get_value(outcome_action_pairs) && get_other_parent()->get_value(reward_parameters))
    {
        for (size_t i = 0; i < outcome_action_pairs.count(); ++i)
        {
            RewardFWDCarActionPair reward_action_pair(fwd_car_reward_calculator->calc_reward(
                                                          &(outcome_action_pairs[i].first),
                                                          &reward_parameters),
                                                      outcome_action_pairs[i].second);
            if (reward_action_pair != val[i])
            {
                return false;
            }
        }
    }
    return true;
}

}
}
}
}
