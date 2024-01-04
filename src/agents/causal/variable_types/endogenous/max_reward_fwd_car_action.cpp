
#include <ori/simcars/agents/causal/variable_types/endogenous/max_reward_fwd_car_action.hpp>

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
        size_t max_reward_index = 0;
        size_t val_index = -1;
        for (size_t i = 0; i < reward_action_pairs.count(); ++i)
        {
            if (reward_action_pairs[i].first > reward_action_pairs[max_reward_index].first)
            {
                max_reward_index = i;
            }
            if (reward_action_pairs[i].second == val)
            {
                val_index = i;
            }
        }

        return max_reward_index == val_index;
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
