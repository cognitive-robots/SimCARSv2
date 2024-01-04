
#include <ori/simcars/agents/causal/variable_types/endogenous/id_goal_val_part.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool IdGoalValPartVariable::get_value(uint64_t &val) const
{
    Goal<uint64_t> goal;
    if (get_parent()->get_value(goal))
    {
        val = goal.val;
        return true;
    }
    else
    {
        return false;
    }
}

bool IdGoalValPartVariable::set_value(uint64_t const &val)
{
    Goal<uint64_t> goal;
    if (get_parent()->get_value(goal))
    {
        goal.val = val;
        return get_parent()->set_value(goal);
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
