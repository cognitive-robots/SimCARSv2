
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/driving_scene_state_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

structures::IArray<IReadOnlyEntityState const*>* AReadOnlyDrivingSceneState::get_entity_states() const
{
    structures::IArray<IReadOnlyDrivingAgentState const*> *driving_agents =
            this->get_driving_agent_states();
    structures::IArray<IReadOnlyEntityState const*> *entities =
            new structures::stl::STLStackArray<IReadOnlyEntityState const*>(driving_agents->count());
    cast_array<IReadOnlyDrivingAgentState const*, IReadOnlyEntityState const*>(*driving_agents, *entities);
    delete driving_agents;
    return entities;
}

IReadOnlyEntityState const* AReadOnlyDrivingSceneState::get_entity_state(std::string const &entity_name) const
{
    return this->get_driving_agent_state(entity_name);
}

}
}
}
