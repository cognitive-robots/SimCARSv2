
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

bool MaxRewardFWDCarActionVariable::get_value(FWDCarAction &val) const
{
    structures::stl::STLStackArray<RewardFWDCarActionPair> reward_action_pairs;
    if (get_parent()->get_value(reward_action_pairs))
    {
        size_t max_reward_index = 0;
        for (size_t i = 1; i < reward_action_pairs.count(); ++i)
        {
            if (reward_action_pairs[i].first > reward_action_pairs[max_reward_index].first)
            {
                max_reward_index = i;
            }
        }

        val = reward_action_pairs[max_reward_index].second;
        return true;
    }
    else
    {
        return false;
    }
}

bool MaxRewardFWDCarActionVariable::set_value(FWDCarAction const &val)
{
    structures::stl::STLStackArray<RewardFWDCarActionPair> reward_action_pairs;
    if (get_parent()->get_value(reward_action_pairs))
    {
        //size_t max_reward_index = 0;
        //size_t val_index = -1;
        for (size_t i = 0; i < reward_action_pairs.count(); ++i)
        {
            FP_DATA_TYPE action_diff = diff(val, reward_action_pairs[i].second);
            FP_DATA_TYPE new_reward = std::exp(-action_diff);
            //FP_DATA_TYPE new_reward = 1.0 / action_diff;
            reward_action_pairs[i].first = new_reward;
            //std::cout << "Action 1 [" << i << "]: " << val << std::endl;
            //std::cout << "Action 2 [" << i << "]: " << reward_action_pairs[i].second << std::endl;
            //std::cout << "Action Diff. [" << i << "]: " << action_diff << std::endl;
            //std::cout << "Action Diff. Reward [" << i << "]: " << new_reward << std::endl;
        }

        return get_parent()->set_value(reward_action_pairs);
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
