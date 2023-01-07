#pragma once

#include <ori/simcars/structures/set_interface.hpp>
#include <ori/simcars/agent/simulation_scene_interface.hpp>
#include <ori/simcars/agent/simulator_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ISimulationSceneFactory
{
public:
    ~ISimulationSceneFactory() = default;

    virtual ISimulationScene* create_simulation_scene(
            IScene *scene,
            ISimulator const *simulator,
            temporal::Duration time_step,
            temporal::Time simulation_start_time,
            temporal::Time simulation_end_time,
            structures::ISet<std::string> *starting_agent_names = nullptr) const = 0;
};

}
}
}
