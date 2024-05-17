
#include <ori/simcars/agents/causal/variable_types/endogenous/max_reward_fwd_car_action.hpp>

#include <cmath>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool MaxRewardFWDCarActionVariable::get_value(FWDCarOutcomeActionPair &val) const
{
    RewardFWDCarOutcomeActionTuples reward_outcome_action_tuples;
    if (get_parent()->get_value(reward_outcome_action_tuples))
    {
        size_t max_reward_index = 0;
        for (size_t i = 1; i < reward_outcome_action_tuples.count(); ++i)
        {
            if (std::get<0>(reward_outcome_action_tuples[i]) >
                    std::get<0>(reward_outcome_action_tuples[max_reward_index]))
            {
                max_reward_index = i;
            }
        }

        val.first = std::get<1>(reward_outcome_action_tuples[max_reward_index]);
        val.second = std::get<2>(reward_outcome_action_tuples[max_reward_index]);
        return true;
    }
    else
    {
        return false;
    }
}

bool MaxRewardFWDCarActionVariable::set_value(FWDCarOutcomeActionPair const &val)
{
    RewardFWDCarOutcomeActionTuples reward_outcome_action_tuples;
    if (get_parent()->get_value(reward_outcome_action_tuples))
    {
        //size_t max_reward_index = 0;
        //size_t val_index = -1;
        for (size_t i = 0; i < reward_outcome_action_tuples.count(); ++i)
        {
            FP_DATA_TYPE action_diff = diff(val.first, std::get<1>(reward_outcome_action_tuples[i]));
            FP_DATA_TYPE new_reward = std::exp(-action_diff);
            //FP_DATA_TYPE new_reward = 1.0 / action_diff;
            std::get<0>(reward_outcome_action_tuples[i]) = new_reward;
            //std::cout << "Action 1 [" << i << "]: " << val.second << std::endl;
            //std::cout << "Outcome 1 [" << i << "]: " << val.first << std::endl;
            //std::cout << "Action 2 [" << i << "]: " << std::get<2>(reward_outcome_action_tuples[i]) << std::endl;
            //std::cout << "Outcome 2 [" << i << "]: " << std::get<1>(reward_outcome_action_tuples[i]) << std::endl;
            //std::cout << "Action Diff. [" << i << "]: " << action_diff << std::endl;
            //std::cout << "Action Diff. Reward [" << i << "]: " << new_reward << std::endl;
        }

        return get_parent()->set_value(reward_outcome_action_tuples);
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
