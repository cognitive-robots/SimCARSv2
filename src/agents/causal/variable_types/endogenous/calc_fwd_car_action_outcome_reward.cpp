
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

bool CalcFWDCarActionOutcomeRewardVariable::get_value(RewardFWDCarOutcomeActionTuples &val) const
{
    FWDCarOutcomeActionPairs outcome_action_pairs;
    FWDCarRewardParameters reward_parameters;
    if (get_endogenous_parent()->get_value(outcome_action_pairs) && get_other_parent()->get_value(reward_parameters))
    {
        val.clear();

        for (size_t i = 0; i < outcome_action_pairs.count(); ++i)
        {
            FP_DATA_TYPE reward = fwd_car_reward_calculator->calc_reward(
                        &(outcome_action_pairs[i].first),
                        &reward_parameters);
            //std::cout << "Action [" << i << "]: " <<  outcome_action_pairs[i].second << std::endl;
            //std::cout << "Outcome [" << i << "]: " <<  outcome_action_pairs[i].first << std::endl;
            //std::cout << "Reward [" << i << "]: " <<  reward << std::endl;
            val.push_back(RewardFWDCarOutcomeActionTuple(reward,
                                                         outcome_action_pairs[i].first,
                                                         outcome_action_pairs[i].second));
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool CalcFWDCarActionOutcomeRewardVariable::set_value(RewardFWDCarOutcomeActionTuples const &val)
{
    // WARNING: Assumes action ordering is the same in both arrays
    // TODO: Potentially consider separating reward weightings from other reward parameters
    FWDCarRewardParameters reward_parameters;
    if (get_other_parent()->get_value(reward_parameters))
    {
        Eigen::MatrixXd individual_rewards(val.count(), 6);
        Eigen::VectorXd combined_rewards(val.count());
        for (size_t i = 0; i < val.count(); ++i)
        {
            FWDCarRewards rewards = fwd_car_reward_calculator->calc_rewards(
                        &(std::get<1>(val[i])), &reward_parameters);
            individual_rewards(i, 0) = rewards.lane_transitions_reward;
            individual_rewards(i, 1) = rewards.dist_headway_reward;
            individual_rewards(i, 2) = rewards.max_speed_reward;
            individual_rewards(i, 3) = rewards.anti_speed_reward;
            individual_rewards(i, 4) = rewards.max_env_force_mag_reward;
            individual_rewards(i, 5) = 1.0;
            /*
            std::cout << "Action [" << i << "]: " << std::get<2>(val[i]) << std::endl;
            std::cout << "Individual Rewards: " << rewards << std::endl;
            std::cout << "Combined Reward: " << std::get<0>(val[i]) << std::endl;
            std::cout << "Lane Transitions: " << std::get<1>(val[i]).lane_transitions << std::endl;
            std::cout << "Max. Env. Force Mag.: " <<
                         std::get<1>(val[i]).max_env_force_mag << std::endl;
            */
            //std::cout << "Max. Env. Force Mag. Reward [" << i <<  "]: " <<
            //             rewards.max_env_force_mag_reward << std::endl;
            combined_rewards(i) = std::get<0>(val[i]);
            //std::cout << "Action Diff. Derived Reward: " << val[i].first << std::endl;
        }
        Eigen::VectorXd reward_weights =
                individual_rewards.colPivHouseholderQr().solve(combined_rewards);
        reward_parameters.lane_transitions_weight = reward_weights(0);
        reward_parameters.dist_headway_weight = reward_weights(1);
        reward_parameters.max_speed_weight = reward_weights(2);
        reward_parameters.anti_speed_weight = reward_weights(3);
        reward_parameters.max_env_force_mag_weight = reward_weights(4);
        reward_parameters.bias_weight = reward_weights(5);

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
