
#include <ori/simcars/agent/driving_simulation_scene_factory.hpp>
#include <ori/simcars/agent/driving_simulation_scene.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

ISimulationScene* DrivingSimulationSceneFactory::create_simulation_scene(
        IScene *scene,
        ISimulator const *simulator,
        temporal::Duration time_step,
        temporal::Time simulation_start_time,
        temporal::Time simulation_end_time,
        structures::ISet<std::string> *starting_agent_names) const
{
    IDrivingScene *driving_scene = dynamic_cast<IDrivingScene*>(scene);
    if (driving_scene == nullptr)
    {
        throw std::invalid_argument("Scene was not a driving scene");
    }

    IDrivingSimulator const *driving_simulator = dynamic_cast<IDrivingSimulator const*>(simulator);
    if (driving_simulator == nullptr)
    {
        throw std::invalid_argument("Simulator was not a driving simulator");
    }

    return DrivingSimulationScene::construct_from(driving_scene, driving_simulator, time_step,
                                                  simulation_start_time, simulation_end_time,
                                                  starting_agent_names);
}

}
}
}
