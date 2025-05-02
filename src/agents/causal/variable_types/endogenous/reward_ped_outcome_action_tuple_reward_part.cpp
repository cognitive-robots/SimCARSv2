
#include <ori/simcars/agents/causal/variable_types/endogenous/reward_ped_outcome_action_tuple_reward_part.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool RewardPedOutcomeActionTupleRewardPartVariable::get_value(FP_DATA_TYPE &val) const
{
    RewardPedOutcomeActionTuple reward_outcome_action_tuple;
    if (get_parent()->get_value(reward_outcome_action_tuple))
    {
        val = std::get<0>(reward_outcome_action_tuple);
        return true;
    }
    else
    {
        return false;
    }
}

bool RewardPedOutcomeActionTupleRewardPartVariable::set_value(FP_DATA_TYPE const &val)
{
    RewardPedOutcomeActionTuple reward_outcome_action_tuple;
    if (get_parent()->get_value(reward_outcome_action_tuple))
    {
        std::get<0>(reward_outcome_action_tuple) = val;
        return get_parent()->set_value(reward_outcome_action_tuple);
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
