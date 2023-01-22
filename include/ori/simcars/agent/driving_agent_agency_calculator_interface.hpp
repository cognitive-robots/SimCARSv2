#pragma once

#include <ori/simcars/agent/driving_agent_state_interface.hpp>
#include <ori/simcars/agent/agency_calculator_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IDrivingAgentAgencyCalculator : public virtual IAgencyCalculator
{
public:
    virtual bool calculate_driving_state_agency(agent::IReadOnlyDrivingAgentState const *state) const = 0;
};

}
}
}
