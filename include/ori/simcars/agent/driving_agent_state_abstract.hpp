#pragma once

#include <ori/simcars/agent/driving_agent_state_interface.hpp>
#include <ori/simcars/agent/read_only_driving_agent_state_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ADrivingAgentState : public virtual AReadOnlyDrivingAgentState, public virtual IDrivingAgentState
{
};

}
}
}
