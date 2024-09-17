
#include <ori/simcars/agents/causal/variable_types/endogenous/max_reward_ped_action.hpp>

#include <cmath>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool MaxRewardPedActionVariable::get_value(PedOutcomeActionPair &val) const
{
    RewardPedOutcomeActionTuples reward_outcome_action_tuples;
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

bool MaxRewardPedActionVariable::set_value(PedOutcomeActionPair const &val)
{
    RewardPedOutcomeActionTuples reward_outcome_action_tuples;
    if (get_parent()->get_value(reward_outcome_action_tuples))
    {
        for (size_t i = 0; i < reward_outcome_action_tuples.count(); ++i)
        {
            FP_DATA_TYPE action_diff = diff(val.first, std::get<1>(reward_outcome_action_tuples[i]));
            FP_DATA_TYPE new_reward = std::exp(-action_diff);
            std::get<0>(reward_outcome_action_tuples[i]) = new_reward;
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
