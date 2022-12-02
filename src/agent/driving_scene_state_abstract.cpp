
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/driving_scene_state_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

structures::IArray<IEntityState*>* ADrivingSceneState::get_mutable_entity_states()
{
    structures::IArray<IDrivingAgentState*> *driving_agents =
            this->get_mutable_driving_agent_states();
    structures::IArray<IEntityState*> *entities =
            new structures::stl::STLStackArray<IEntityState*>(driving_agents->count());
    cast_array<IDrivingAgentState*, IEntityState*>(*driving_agents, *entities);
    delete driving_agents;
    return entities;
}

IEntityState* ADrivingSceneState::get_mutable_entity_state(std::string const &entity_name)
{
    return this->get_mutable_driving_agent_state(entity_name);
}

}
}
}
