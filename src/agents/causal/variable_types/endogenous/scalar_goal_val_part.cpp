
#include <ori/simcars/agents/causal/variable_types/endogenous/scalar_goal_val_part.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

FP_DATA_TYPE ScalarGoalValPartVariable::get_value() const
{
    return get_parent()->get_value().val;
}

}
}
}
}
