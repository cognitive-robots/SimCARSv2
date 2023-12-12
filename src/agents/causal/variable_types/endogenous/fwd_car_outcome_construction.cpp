
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_outcome_construction.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

FWDCarOutcome FWDCarOutcomeConstructionVariable::get_value() const
{
    FWDCarOutcome outcome;

    outcome.lane_transitions = get_endogenous_parent_1()->get_value();
    outcome.final_speed = get_endogenous_parent_2()->get_value();
    outcome.max_env_force_mag = get_other_parent()->get_value();

    return outcome;
}

}
}
}
}
