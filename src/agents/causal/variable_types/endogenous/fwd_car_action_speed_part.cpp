
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_speed_part.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool FWDCarActionSpeedPartVariable::get_value(Goal<FP_DATA_TYPE> &val) const
{
    FWDCarAction action;
    if (get_parent()->get_value(action))
    {
        val = action.speed_goal;
        return true;
    }
    else
    {
        return false;
    }
}

bool FWDCarActionSpeedPartVariable::set_value(Goal<FP_DATA_TYPE> const &val)
{
    FWDCarAction action;
    if (get_parent()->get_value(action))
    {
        action.speed_goal = val;
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
