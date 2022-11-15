#pragma once

#include <ori/simcars/agent/scene_interface.hpp>
#include <ori/simcars/agent/driving_agent_controller_interface.hpp>
#include <ori/simcars/agent/driving_simulator_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class BasicDrivingSimulator : public virtual IDrivingSimulator
{
    IDrivingAgentController const *controller;

public:
    BasicDrivingSimulator(IDrivingAgentController const *controller);

    void simulate(agent::IState const *current_state, agent::IState *next_state, temporal::Duration time_step) const override;

    void simulate_driving_scene(agent::IDrivingSceneState const *current_state, agent::IDrivingSceneState *next_state, temporal::Duration time_step) const override;
    void simulate_driving_agent(agent::IDrivingAgentState const *current_state, agent::IDrivingAgentState *next_state, temporal::Duration time_step) const override;
};

}
}
}
