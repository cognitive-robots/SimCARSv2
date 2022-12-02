
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/view_driving_scene_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

ViewDrivingSceneState::ViewDrivingSceneState(IDrivingScene *scene, temporal::Time time)
    : scene(scene), time(time)
{
}

structures::IArray<IReadOnlyDrivingAgentState const*>* ViewDrivingSceneState::get_driving_agent_states() const
{
    structures::IArray<IDrivingAgent const*> *driving_agents = scene->get_driving_agents();
    structures::IStackArray<IReadOnlyDrivingAgentState const*> *driving_agent_states =
            new structures::stl::STLStackArray<IReadOnlyDrivingAgentState const*>;
    for (size_t i = 0; i < driving_agents->count(); ++i)
    {
        if ((*driving_agents)[i]->is_state_available(time))
        {
            driving_agent_states->push_back((*driving_agents)[i]->get_driving_agent_state(time));
        }
    }
    delete driving_agents;
    return driving_agent_states;
}

IReadOnlyDrivingAgentState const* ViewDrivingSceneState::get_driving_agent_state(std::string const &driving_agent_name) const
{
    return scene->get_driving_agent(driving_agent_name)->get_driving_agent_state(time);
}

IDrivingScene const* ViewDrivingSceneState::get_scene() const
{
    return scene;
}

temporal::Time ViewDrivingSceneState::get_time() const
{
    return time;
}

structures::IArray<IDrivingAgentState*>* ViewDrivingSceneState::get_mutable_driving_agent_states()
{
    structures::IArray<IDrivingAgent*> *driving_agents = scene->get_mutable_driving_agents();
    structures::IStackArray<IDrivingAgentState*> *driving_agent_states =
            new structures::stl::STLStackArray<IDrivingAgentState*>;
    for (size_t i = 0; i < driving_agents->count(); ++i)
    {
        if ((*driving_agents)[i]->is_state_available(time))
        {
            driving_agent_states->push_back((*driving_agents)[i]->get_mutable_driving_agent_state(time));
        }
    }
    delete driving_agents;
    return driving_agent_states;
}

IDrivingAgentState* ViewDrivingSceneState::get_mutable_driving_agent_state(std::string const &driving_agent_name)
{
    return scene->get_mutable_driving_agent(driving_agent_name)->get_mutable_driving_agent_state(time);
}

void ViewDrivingSceneState::set_scene(IDrivingScene *scene)
{
    this->scene = scene;
}

void ViewDrivingSceneState::set_time(temporal::Time time)
{
    this->time = time;
}

}
}
}
