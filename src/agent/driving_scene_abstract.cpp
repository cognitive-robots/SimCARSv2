
#include <ori/simcars/agent/driving_scene_abstract.hpp>
#include <ori/simcars/agent/basic_driving_scene_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

IState* ADrivingScene::get_state(temporal::Time time) const
{
    return this->get_driving_scene_state(time);
}

IDrivingSceneState* ADrivingScene::get_driving_scene_state(temporal::Time time) const
{
    IDrivingSceneState *driving_scene_state = new BasicDrivingSceneState;

    structures::IArray<IDrivingAgent const*> *driving_agents = this->get_driving_agents();

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

    delete driving_agents;

    return driving_scene_state;
}

}
}
}
