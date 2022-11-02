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
    std::shared_ptr<const IDrivingAgentController> controller;

public:
    BasicDrivingSimulator(std::shared_ptr<const IDrivingAgentController> controller);

    void simulate(std::shared_ptr<const agent::IState> current_state, std::shared_ptr<agent::IState> next_state, temporal::Duration time_step) const override;

    void simulate_driving_scene(std::shared_ptr<const agent::IDrivingSceneState> current_state, std::shared_ptr<agent::IDrivingSceneState> next_state, temporal::Duration time_step) const override;
    void simulate_driving_agent(std::shared_ptr<const agent::IDrivingAgentState> current_state, std::shared_ptr<agent::IDrivingAgentState> next_state, temporal::Duration time_step) const override;
};

}
}
}
