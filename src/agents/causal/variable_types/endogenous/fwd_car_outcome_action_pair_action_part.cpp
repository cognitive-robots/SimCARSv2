
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_outcome_action_pair_action_part.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool FWDCarOutcomeActionPairActionPartVariable::get_value(FWDCarAction &val) const
{
    FWDCarOutcomeActionPair outcome_action_pair;
    if (get_parent()->get_value(outcome_action_pair))
    {
        val = outcome_action_pair.second;
        return true;
    }
    else
    {
        return false;
    }
}

bool FWDCarOutcomeActionPairActionPartVariable::set_value(FWDCarAction const &val)
{
    FWDCarOutcomeActionPair outcome_action_pair;
    if (get_parent()->get_value(outcome_action_pair))
    {
        outcome_action_pair.second = val;
        return get_parent()->set_value(outcome_action_pair);
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
