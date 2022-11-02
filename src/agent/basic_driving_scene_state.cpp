
#include <ori/simcars/structures/stl/stl_concat_array.hpp>
#include <ori/simcars/agent/basic_driving_scene_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

BasicDrivingSceneState::BasicDrivingSceneState() {}

BasicDrivingSceneState::BasicDrivingSceneState(std::shared_ptr<const IDrivingSceneState> driving_scene_state)
{
    std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgentState>>> driving_agent_states =
            driving_scene_state->get_driving_agent_states();
    for (size_t i = 0; i < driving_agent_states->count(); ++i)
    {
        driving_agent_state_dict.update((*driving_agent_states)[i]->get_driving_agent_name(), (*driving_agent_states)[i]);
    }
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> BasicDrivingSceneState::get_parameter_values() const
{
    std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgentState>>> driving_agent_states = this->get_driving_agent_states();

    std::shared_ptr<structures::stl::STLConcatArray<std::shared_ptr<const IValuelessConstant>>> parameter_values(
                new structures::stl::STLConcatArray<std::shared_ptr<const IValuelessConstant>>(driving_agent_states->count()));

    size_t i;
    for(i = 0; i < driving_agent_states->count(); ++i)
    {
        parameter_values->get_array(i) = (*driving_agent_states)[i]->get_parameter_values();
    }

    return parameter_values;
}

std::shared_ptr<const IValuelessConstant> BasicDrivingSceneState::get_parameter_value(const std::string& parameter_name) const
{
    const std::string entity_name = parameter_name.substr(0, parameter_name.find('.'));
    const std::shared_ptr<const IDrivingAgentState> driving_agent_state = get_driving_agent_state(entity_name);
    return driving_agent_state->get_parameter_value(parameter_name);
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgentState>>> BasicDrivingSceneState::get_driving_agent_states() const
{
    return std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgentState>>>(
            new structures::stl::STLStackArray<std::shared_ptr<const IDrivingAgentState>>(driving_agent_state_dict.get_values()));
}

std::shared_ptr<const IDrivingAgentState> BasicDrivingSceneState::get_driving_agent_state(const std::string& driving_agent_name) const
{
    return driving_agent_state_dict[driving_agent_name];
}

void BasicDrivingSceneState::set_driving_agent_states(
        std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgentState>>> driving_agent_states)
{
    for (size_t i = 0; i < driving_agent_states->count(); ++i)
    {
        this->set_driving_agent_state((*driving_agent_states)[i]);
    }
}

void BasicDrivingSceneState::set_driving_agent_state(std::shared_ptr<const IDrivingAgentState> driving_agent_state)
{
    driving_agent_state_dict.update(driving_agent_state->get_driving_agent_name(), driving_agent_state);
}

}
}
}
