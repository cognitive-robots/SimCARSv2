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
    virtual void simulate_driving_scene(std::shared_ptr<const agent::IDrivingSceneState> current_state, std::shared_ptr<agent::IDrivingSceneState> next_state, temporal::Duration time_step) const = 0;
    virtual void simulate_driving_agent(std::shared_ptr<const agent::IDrivingAgentState> current_state, std::shared_ptr<agent::IDrivingAgentState> next_state, temporal::Duration time_step) const = 0;
};

}
}
}
