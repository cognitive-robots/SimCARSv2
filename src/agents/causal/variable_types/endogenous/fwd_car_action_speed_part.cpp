
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_speed_part.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

Goal<FP_DATA_TYPE> FWDCarActionSpeedPartVariable::get_value() const
{
    return get_parent()->get_value().speed_goal;
}

}
}
}
}
