
#include <ori/simcars/agent/basic_driving_agent_agency_calculator.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

bool BasicDrivingAgentAgencyCalculator::calculate_driving_state_agency(IReadOnlyDrivingAgentState const *state) const
{
    return state->get_cumilative_collision_time_variable()->get_value().count() == 0;
}

}
}
}
