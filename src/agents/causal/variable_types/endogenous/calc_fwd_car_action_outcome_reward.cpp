
#include <ori/simcars/agents/causal/variable_types/endogenous/calc_fwd_car_action_outcome_reward.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

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
    // WARNING: Assumes action ordering is the same in both arrays
    // TODO: Potentially consider separating reward weightings from other reward parameters
    structures::stl::STLStackArray<FWDCarOutcomeActionPair> outcome_action_pairs;
    FWDCarRewardParameters reward_parameters;
    if (get_endogenous_parent()->get_value(outcome_action_pairs) &&
            get_other_parent()->get_value(reward_parameters))
    {
        Eigen::MatrixXd individual_rewards(outcome_action_pairs.count(), 3);
        Eigen::VectorXd combined_rewards(outcome_action_pairs.count());
        for (size_t i = 0; i < outcome_action_pairs.count(); ++i)
        {
            FWDCarRewards rewards = fwd_car_reward_calculator->calc_rewards(
                        &(outcome_action_pairs[i].first),
                        &reward_parameters);
            individual_rewards(i, 0) = rewards.lane_transitions_reward;
            individual_rewards(i, 1) = rewards.final_speed_reward;
            individual_rewards(i, 2) = rewards.max_env_force_mag_reward;
            combined_rewards(i) = val[i].first;
        }
        Eigen::VectorXd reward_weights =
                individual_rewards.colPivHouseholderQr().solve(combined_rewards);
        reward_parameters.lane_transitions_weight = reward_weights(0);
        reward_parameters.final_speed_weight = reward_weights(1);
        reward_parameters.max_env_force_mag_weight = reward_weights(2);

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
