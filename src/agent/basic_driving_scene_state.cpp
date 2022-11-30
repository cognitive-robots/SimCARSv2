
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_concat_array.hpp>
#include <ori/simcars/agent/basic_driving_scene_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

BasicDrivingSceneState::BasicDrivingSceneState(bool delete_dicts) :
    delete_dicts(delete_dicts) {}

BasicDrivingSceneState::BasicDrivingSceneState(
        IDrivingSceneState const *driving_scene_state, bool copy_parameters) :
    delete_dicts(copy_parameters)
{
    structures::IArray<IDrivingAgentState const*> *driving_agent_states =
            driving_scene_state->get_driving_agent_states();
    for (size_t i = 0; i < driving_agent_states->count(); ++i)
    {
        driving_agent_state_dict.update((*driving_agent_states)[i]->get_driving_agent_name(), (*driving_agent_states)[i]);
    }
}

BasicDrivingSceneState::~BasicDrivingSceneState()
{
    if (delete_dicts)
    {
        structures::IArray<IDrivingAgentState const*> const *driving_agent_states = driving_agent_state_dict.get_values();

        for (size_t i = 0; i < driving_agent_states->count(); ++i)
        {
            delete (*driving_agent_states)[i];
        }
    }
}

structures::IArray<IDrivingAgentState const*>* BasicDrivingSceneState::get_driving_agent_states() const
{
    structures::IStackArray<IDrivingAgentState const*> *driving_agent_states =
            new structures::stl::STLStackArray<IDrivingAgentState const*>(driving_agent_state_dict.count());
    driving_agent_state_dict.get_values(driving_agent_states);
    return driving_agent_states;
}

IDrivingAgentState const* BasicDrivingSceneState::get_driving_agent_state(std::string const &driving_agent_name) const
{
    return driving_agent_state_dict[driving_agent_name];
}

void BasicDrivingSceneState::set_driving_agent_states(
        structures::IArray<IDrivingAgentState const*> *driving_agent_states)
{
    for (size_t i = 0; i < driving_agent_states->count(); ++i)
    {
        this->set_driving_agent_state((*driving_agent_states)[i]);
    }
}

void BasicDrivingSceneState::set_driving_agent_state(IDrivingAgentState const *driving_agent_state)
{
    if (driving_agent_state_dict.contains(driving_agent_state->get_driving_agent_name()))
    {
        delete driving_agent_state_dict[driving_agent_state->get_driving_agent_name()];
    }
    driving_agent_state_dict.update(driving_agent_state->get_driving_agent_name(), driving_agent_state);
}

}
}
}
