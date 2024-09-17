
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_action_node_part.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool PedActionNodePartVariable::get_value(Goal<uint64_t> &val) const
{
    PedAction action;
    if (get_parent()->get_value(action))
    {
        val = action.node_goal;
        return true;
    }
    else
    {
        return false;
    }
}

bool PedActionNodePartVariable::set_value(Goal<uint64_t> const &val)
{
    PedAction action;
    if (get_parent()->get_value(action))
    {
        action.node_goal = val;
        return get_parent()->set_value(action);
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
