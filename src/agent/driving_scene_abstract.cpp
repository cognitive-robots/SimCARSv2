
#include <ori/simcars/agent/driving_scene_abstract.hpp>
#include <ori/simcars/agent/basic_driving_scene_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

std::shared_ptr<IState> ADrivingScene::get_state(temporal::Time time) const
{
    return this->get_driving_scene_state(time);
}

std::shared_ptr<IDrivingSceneState> ADrivingScene::get_driving_scene_state(temporal::Time time) const
{
    std::shared_ptr<IDrivingSceneState> driving_scene_state(new BasicDrivingSceneState());

    std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgent>>> driving_agents = this->get_driving_agents();

    for (size_t i = 0; i < driving_agents->count(); ++i)
    {
        try
        {
            driving_scene_state->set_driving_agent_state((*driving_agents)[i]->get_driving_agent_state(time));
        }
        catch (std::out_of_range)
        {
            // Driving agent is not present for this timestamp
        }
    }

    return driving_scene_state;
}

}
}
}
