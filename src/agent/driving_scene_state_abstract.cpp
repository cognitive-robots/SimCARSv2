
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/driving_scene_state_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

structures::IArray<IEntityState const*>* ADrivingSceneState::get_entity_states() const
{
    structures::IArray<IDrivingAgentState const*> *driving_agents =
            this->get_driving_agent_states();
    structures::IArray<IEntityState const*> *entities =
            new structures::stl::STLStackArray<IEntityState const*>(driving_agents->count());
    cast_array<IDrivingAgentState const*, IEntityState const*>(*driving_agents, *entities);
    return entities;
}

IEntityState const* ADrivingSceneState::get_entity_state(std::string const &entity_name) const
{
    return this->get_driving_agent_state(entity_name);
}

void ADrivingSceneState::set_entity_states(structures::IArray<IEntityState const*> const *entities)
{
    for (size_t i = 0; i < entities->count(); ++i)
    {
        this->set_entity_state((*entities)[i]);
    }
}

void ADrivingSceneState::set_entity_state(IEntityState const *entity)
{
    this->set_driving_agent_state(dynamic_cast<IDrivingAgentState const*>(entity));
}

}
}
}
