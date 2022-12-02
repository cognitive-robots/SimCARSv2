#pragma once

#include <ori/simcars/agent/read_only_scene_state_interface.hpp>
#include <ori/simcars/agent/driving_agent_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IReadOnlyDrivingSceneState : public virtual IReadOnlySceneState
{
public:
    virtual structures::IArray<IReadOnlyDrivingAgentState const*>* get_driving_agent_states() const = 0;
    virtual IReadOnlyDrivingAgentState const* get_driving_agent_state(std::string const &driving_agent_name) const = 0;
};

}
}
}
