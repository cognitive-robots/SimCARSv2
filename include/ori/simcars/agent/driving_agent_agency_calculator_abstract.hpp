#pragma once

#include <ori/simcars/agent/driving_agent_agency_calculator_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ADrivingAgentAgencyCalculator : public virtual IDrivingAgentAgencyCalculator
{
public:
    bool calculate_state_agency(agent::IReadOnlyEntityState const *state) const override;
};

}
}
}
