#pragma once

#include <ori/simcars/agent/controller_interface.hpp>
#include <ori/simcars/agent/driving_agent_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IDrivingAgentController : public virtual IController
{
public:
    virtual void modify_driving_agent_state(agent::IReadOnlyDrivingAgentState const *original_state, agent::IDrivingAgentState *modified_state) const = 0;
};

}
}
}
