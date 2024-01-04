
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_lane_part.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool FWDCarActionLanePartVariable::get_value(Goal<uint64_t> &val) const
{
    FWDCarAction action;
    if (get_parent()->get_value(action))
    {
        val = action.lane_goal;
        return true;
    }
    else
    {
        return false;
    }
}

bool FWDCarActionLanePartVariable::set_value(Goal<uint64_t> const &val)
{
    FWDCarAction action;
    if (get_parent()->get_value(action))
    {
        action.lane_goal = val;
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
