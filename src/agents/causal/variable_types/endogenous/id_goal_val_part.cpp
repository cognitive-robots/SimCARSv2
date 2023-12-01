
#include <ori/simcars/agents/causal/variable_types/endogenous/id_goal_val_part.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

uint64_t IdGoalValPartVariable::get_value() const
{
    return get_parent()->get_value().val;
}

}
}
}
}
