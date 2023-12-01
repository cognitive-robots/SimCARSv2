
#include <ori/simcars/agents/causal/variable_types/endogenous/id_goal_time_part.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

temporal::Time IdGoalTimePartVariable::get_value() const
{
    return get_parent()->get_value().time;
}

}
}
}
}
