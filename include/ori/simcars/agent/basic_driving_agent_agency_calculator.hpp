#pragma once

#include <ori/simcars/agent/driving_agent_agency_calculator_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class BasicDrivingAgentAgencyCalculator : public virtual ADrivingAgentAgencyCalculator
{
public:
    bool calculate_driving_state_agency(agent::IReadOnlyDrivingAgentState const *state) const override;
};

}
}
}
