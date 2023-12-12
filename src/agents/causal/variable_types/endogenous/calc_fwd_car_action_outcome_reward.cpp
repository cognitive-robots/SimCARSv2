
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
        simcars::causal::IEndogenousVariable<structures::stl::STLStackArray<FWDCarOutcomeActionPair>> const *endogenous_parent,
        simcars::causal::IVariable<FWDCarRewardParameters> const *other_parent,
        IFWDCarRewardCalc const *fwd_car_reward_calculator) :
    ABinaryEndogenousVariable(endogenous_parent, other_parent),
    fwd_car_reward_calculator(fwd_car_reward_calculator) {}

structures::stl::STLStackArray<RewardFWDCarActionPair> CalcFWDCarActionOutcomeRewardVariable::get_value() const
{
    structures::stl::STLStackArray<FWDCarOutcomeActionPair> outcome_action_pairs =
            get_endogenous_parent()->get_value();
    FWDCarRewardParameters reward_parameters = get_other_parent()->get_value();

    structures::stl::STLStackArray<RewardFWDCarActionPair> reward_action_pairs;

    for (size_t i = 0; i < reward_action_pairs.count(); ++i)
    {
        reward_action_pairs.push_back(
                    RewardFWDCarActionPair(fwd_car_reward_calculator->calc_reward(
                                               &(outcome_action_pairs[i].first),
                                               &reward_parameters),
                                           outcome_action_pairs[i].second));
    }

    return reward_action_pairs;
}

}
}
}
}
