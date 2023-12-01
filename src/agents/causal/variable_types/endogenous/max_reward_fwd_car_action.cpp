
#include <ori/simcars/agents/causal/variable_types/endogenous/max_reward_fwd_car_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

FWDCarAction MaxRewardFWDCarAction::get_value() const
{
    structures::stl::STLStackArray<RewardFWDCarActionPair> reward_action_pairs =
            get_parent()->get_value();

    size_t max_reward_index = 0;
    for (size_t i = 1; i < reward_action_pairs.count(); ++i)
    {
        if (reward_action_pairs[i].first > reward_action_pairs[max_reward_index].first)
        {
            max_reward_index = i;
        }
    }

    return reward_action_pairs[max_reward_index].second;
}

}
}
}
}
