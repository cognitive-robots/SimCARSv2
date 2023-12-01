
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_lane_part.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

Goal<uint64_t> FWDCarActionLanePart::get_value() const
{
    return get_parent()->get_value().lane_goal;
}

}
}
}
}
