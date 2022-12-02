#pragma once

#include <ori/simcars/agent/simulator_interface.hpp>
#include <ori/simcars/agent/driving_scene_state_interface.hpp>
#include <ori/simcars/agent/driving_agent_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IDrivingSimulator : public virtual ISimulator
{
public:
    virtual void simulate_driving_scene(agent::IReadOnlyDrivingSceneState const *current_state, agent::IDrivingSceneState *next_state, temporal::Duration time_step) const = 0;
    virtual void simulate_driving_agent(agent::IReadOnlyDrivingAgentState const *current_state, agent::IDrivingAgentState *next_state, temporal::Duration time_step) const = 0;
};

}
}
}
