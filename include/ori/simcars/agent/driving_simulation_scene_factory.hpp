#pragma once

#include <ori/simcars/agent/simulation_scene_factory_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class DrivingSimulationSceneFactory : public ISimulationSceneFactory
{
public:
    ISimulationScene* create_simulation_scene(
            IScene *scene,
            ISimulator const *simulator,
            temporal::Duration time_step,
            temporal::Time simulation_start_time,
            temporal::Time simulation_end_time,
            structures::ISet<std::string> *starting_agent_names) const override;
};

}
}
}
