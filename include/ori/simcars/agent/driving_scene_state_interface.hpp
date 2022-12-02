#pragma once

#include <ori/simcars/agent/scene_state_interface.hpp>
#include <ori/simcars/agent/read_only_driving_scene_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IDrivingSceneState : public virtual IReadOnlyDrivingSceneState, public virtual ISceneState
{
public:
    virtual structures::IArray<IDrivingAgentState*>* get_mutable_driving_agent_states() = 0;
    virtual IDrivingAgentState* get_mutable_driving_agent_state(std::string const &driving_agent_name) = 0;
};

}
}
}
