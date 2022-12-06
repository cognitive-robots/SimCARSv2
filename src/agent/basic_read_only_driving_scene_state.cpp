
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_concat_array.hpp>
#include <ori/simcars/agent/basic_read_only_driving_scene_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

BasicReadOnlyDrivingSceneState::BasicReadOnlyDrivingSceneState(IDrivingScene const *scene, temporal::Time time, bool delete_dicts) :
    time(time), delete_dicts(delete_dicts)
{
    structures::IArray<IDrivingAgent const*> *driving_agents = scene->get_driving_agents();

    for (size_t i = 0; i < driving_agents->count(); ++i)
    {
        try
        {
            driving_agent_state_dict.update((*driving_agents)[i]->get_name(), (*driving_agents)[i]->get_driving_agent_state(time));
        }
        catch (std::out_of_range)
        {
            // Driving agent is not present for this timestamp
        }
    }

    delete driving_agents;
}

BasicReadOnlyDrivingSceneState::BasicReadOnlyDrivingSceneState(
        IReadOnlyDrivingSceneState *driving_scene_state, bool copy_parameters) :
    time(driving_scene_state->get_time()), delete_dicts(copy_parameters)
{
    structures::IArray<IReadOnlyDrivingAgentState const*> *driving_agent_states =
            driving_scene_state->get_driving_agent_states();
    for (size_t i = 0; i < driving_agent_states->count(); ++i)
    {
        driving_agent_state_dict.update((*driving_agent_states)[i]->get_name(), (*driving_agent_states)[i]);
    }
}

BasicReadOnlyDrivingSceneState::~BasicReadOnlyDrivingSceneState()
{
    if (delete_dicts)
    {
        structures::IArray<IReadOnlyDrivingAgentState const*> const *driving_agent_states = driving_agent_state_dict.get_values();

        for (size_t i = 0; i < driving_agent_states->count(); ++i)
        {
            delete (*driving_agent_states)[i];
        }
    }
}

temporal::Time BasicReadOnlyDrivingSceneState::get_time() const
{
    return time;
}

structures::IArray<IReadOnlyDrivingAgentState const*>* BasicReadOnlyDrivingSceneState::get_driving_agent_states() const
{
    structures::IStackArray<IReadOnlyDrivingAgentState const*> *driving_agent_states =
            new structures::stl::STLStackArray<IReadOnlyDrivingAgentState const*>(driving_agent_state_dict.count());
    driving_agent_state_dict.get_values(driving_agent_states);
    return driving_agent_states;
}

IReadOnlyDrivingAgentState const* BasicReadOnlyDrivingSceneState::get_driving_agent_state(std::string const &driving_agent_name) const
{
    return driving_agent_state_dict[driving_agent_name];
}

}
}
}
