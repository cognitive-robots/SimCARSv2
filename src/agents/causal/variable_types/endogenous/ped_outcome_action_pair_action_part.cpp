
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_outcome_action_pair_action_part.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool PedOutcomeActionPairActionPartVariable::get_value(PedAction &val) const
{
    PedOutcomeActionPair outcome_action_pair;
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

bool PedOutcomeActionPairActionPartVariable::set_value(PedAction const &val)
{
    PedOutcomeActionPair outcome_action_pair;
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
