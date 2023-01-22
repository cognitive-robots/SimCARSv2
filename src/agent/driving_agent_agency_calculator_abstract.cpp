
#include <ori/simcars/agent/driving_agent_agency_calculator_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

bool ADrivingAgentAgencyCalculator::calculate_state_agency(IReadOnlyEntityState const *state) const
{
    return this->calculate_driving_state_agency(dynamic_cast<IReadOnlyDrivingAgentState const*>(state));
}

}
}
}
