
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_outcome_construction.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool FWDCarOutcomeConstructionVariable::get_value(FWDCarOutcome &val) const
{
    FP_DATA_TYPE lane_transitions;
    FP_DATA_TYPE final_speed;
    FP_DATA_TYPE max_env_force_mag;
    if (get_endogenous_parent_1()->get_value(lane_transitions) &&
            get_endogenous_parent_2()->get_value(final_speed) &&
            get_other_parent()->get_value(max_env_force_mag))
    {
        val.lane_transitions = lane_transitions;
        val.final_speed = final_speed;
        val.max_env_force_mag = max_env_force_mag;

        return true;
    }
    else
    {
        return false;
    }
}

bool FWDCarOutcomeConstructionVariable::set_value(FWDCarOutcome const &val)
{
    return get_endogenous_parent_1()->set_value(val.lane_transitions) &&
            get_endogenous_parent_2()->set_value(val.final_speed) &&
            get_other_parent()->set_value(val.max_env_force_mag);
}

}
}
}
}
