
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_concat_array.hpp>
#include <ori/simcars/agent/basic_driving_scene_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

BasicDrivingSceneState::BasicDrivingSceneState() {}

BasicDrivingSceneState::BasicDrivingSceneState(IDrivingSceneState const *driving_scene_state)
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
    structures::IArray<IDrivingAgentState const*> const *driving_agent_states = driving_agent_state_dict.get_values();

    for (size_t i = 0; i < driving_agent_states->count(); ++i)
    {
        delete (*driving_agent_states)[i];
    }
}

structures::IArray<IValuelessConstant const*>* BasicDrivingSceneState::get_parameter_values() const
{
    structures::IArray<IDrivingAgentState const*> *driving_agent_states = this->get_driving_agent_states();

    structures::stl::STLConcatArray<IValuelessConstant const*> *parameter_values =
                new structures::stl::STLConcatArray<IValuelessConstant const*>(driving_agent_states->count());

    size_t i;
    for(i = 0; i < driving_agent_states->count(); ++i)
    {
        parameter_values->get_array(i) = (*driving_agent_states)[i]->get_parameter_values();
    }

    return parameter_values;
}

IValuelessConstant const* BasicDrivingSceneState::get_parameter_value(std::string const &parameter_name) const
{
    std::string const entity_name = parameter_name.substr(0, parameter_name.find('.'));
    IDrivingAgentState const* const driving_agent_state = get_driving_agent_state(entity_name);
    return driving_agent_state->get_parameter_value(parameter_name);
}

void BasicDrivingSceneState::set_parameter_values(structures::IArray<IValuelessConstant const*> const *parameter_values)
{
    for (size_t i = 0; i < parameter_values->count(); ++i)
    {
        set_parameter_value((*parameter_values)[i]);
    }
}

void BasicDrivingSceneState::set_parameter_value(const IValuelessConstant *parameter_value)
{
    throw utils::NotImplementedException();
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
