
#include <ori/simcars/agents/causal/variable_types/endogenous/id_goal_time_part.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool IdGoalTimePartVariable::get_value(temporal::Time &val) const
{
    Goal<uint64_t> goal;
    if (get_parent()->get_value(goal))
    {
        val = goal.time;
        return true;
    }
    else
    {
        return false;
    }
}

bool IdGoalTimePartVariable::set_value(temporal::Time const &val)
{
    Goal<uint64_t> goal;
    if (get_parent()->get_value(goal))
    {
        goal.time = val;
        return get_parent()->set_value(goal);
    }
    else
    {
        return false;
    }
}

}
}
}
}
