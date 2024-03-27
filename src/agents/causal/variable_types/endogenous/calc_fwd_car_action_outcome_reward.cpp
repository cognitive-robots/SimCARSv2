
#include <ori/simcars/agents/causal/variable_types/endogenous/calc_fwd_car_action_outcome_reward.hpp>

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

CalcFWDCarActionOutcomeRewardVariable::CalcFWDCarActionOutcomeRewardVariable(
        simcars::causal::IEndogenousVariable<FWDCarOutcomeActionPairs> *endogenous_parent,
        simcars::causal::IVariable<FWDCarRewardParameters> *other_parent,
        IFWDCarRewardCalc const *fwd_car_reward_calculator) :
    ABinaryEndogenousVariable(endogenous_parent, other_parent),
    fwd_car_reward_calculator(fwd_car_reward_calculator) {}

bool CalcFWDCarActionOutcomeRewardVariable::get_value(structures::stl::STLStackArray<RewardFWDCarActionPair> &val) const
{
    FWDCarOutcomeActionPairs outcome_action_pairs;
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
    FWDCarOutcomeActionPairs outcome_action_pairs;
    FWDCarRewardParameters reward_parameters;
    if (get_endogenous_parent()->get_value(outcome_action_pairs) &&
            get_other_parent()->get_value(reward_parameters))
    {
        Eigen::MatrixXd individual_rewards(outcome_action_pairs.count(), 5);
        Eigen::VectorXd combined_rewards(outcome_action_pairs.count());
        for (size_t i = 0; i < outcome_action_pairs.count(); ++i)
        {
            FWDCarRewards rewards = fwd_car_reward_calculator->calc_rewards(
                        &(outcome_action_pairs[i].first),
                        &reward_parameters);
            individual_rewards(i, 0) = rewards.lane_transitions_reward;
            individual_rewards(i, 1) = rewards.caution_reward;
            individual_rewards(i, 2) = rewards.speed_limit_excess_reward;
            individual_rewards(i, 3) = rewards.max_env_force_mag_reward;
            individual_rewards(i, 4) = 1.0;
            std::cout << "Action [" << i << "]: " << outcome_action_pairs[i].second << std::endl;
            std::cout << "Individual Rewards: " << rewards << std::endl;
            std::cout << "Combined Reward: " << val[i].first << std::endl;
            std::cout << "Max. Env. Force Mag. [" << i << "]: " <<
                         outcome_action_pairs[i].first.max_env_force_mag << std::endl;
            //std::cout << "Max. Env. Force Mag. Reward [" << i <<  "]: " <<
            //             rewards.max_env_force_mag_reward << std::endl;
            combined_rewards(i) = val[i].first;
            //std::cout << "Action Diff. Derived Reward: " << val[i].first << std::endl;
        }
        Eigen::VectorXd reward_weights =
                individual_rewards.colPivHouseholderQr().solve(combined_rewards);
        reward_parameters.lane_transitions_weight = reward_weights(0);
        reward_parameters.caution_weight = reward_weights(1);
        reward_parameters.speed_limit_excess_weight = reward_weights(2);
        reward_parameters.max_env_force_mag_weight = reward_weights(3);
        reward_parameters.bias_weight = reward_weights(4);

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
