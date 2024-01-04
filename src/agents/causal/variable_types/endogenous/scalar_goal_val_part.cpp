
#include <ori/simcars/agents/causal/variable_types/endogenous/scalar_goal_val_part.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool ScalarGoalValPartVariable::get_value(FP_DATA_TYPE &val) const
{
    Goal<FP_DATA_TYPE> goal;
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

bool ScalarGoalValPartVariable::set_value(FP_DATA_TYPE const &val)
{
    Goal<FP_DATA_TYPE> goal;
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
