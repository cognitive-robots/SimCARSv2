
#include <ori/simcars/agent/driving_scene_abstract.hpp>
#include <ori/simcars/agent/basic_driving_scene_state.hpp>
#include <ori/simcars/agent/basic_read_only_driving_scene_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

structures::IArray<IEntity const*>* ADrivingScene::get_entities() const
{
    structures::IArray<IDrivingAgent const*>* const driving_agents = this->get_driving_agents();
    structures::IArray<IEntity const*>* const entities =
            new structures::stl::STLStackArray<IEntity const*>(driving_agents->count());
    cast_array(*driving_agents, *entities);
    delete driving_agents;
    return entities;
}

IEntity const* ADrivingScene::get_entity(std::string const &entity_name) const
{
    return this->get_driving_agent(entity_name);
}

IReadOnlySceneState const* ADrivingScene::get_state(temporal::Time time) const
{
    return this->get_driving_scene_state(time);
}

IReadOnlyDrivingSceneState const* ADrivingScene::get_driving_scene_state(temporal::Time time) const
{
    return new BasicReadOnlyDrivingSceneState(this, time);
}

structures::IArray<IEntity*>* ADrivingScene::get_mutable_entities()
{
    structures::IArray<IDrivingAgent*>* const driving_agents = this->get_mutable_driving_agents();
    structures::IArray<IEntity*>* const entities =
            new structures::stl::STLStackArray<IEntity*>(driving_agents->count());
    cast_array(*driving_agents, *entities);
    delete driving_agents;
    return entities;
}

IEntity* ADrivingScene::get_mutable_entity(std::string const &entity_name)
{
    return this->get_mutable_driving_agent(entity_name);
}

ISceneState* ADrivingScene::get_mutable_state(temporal::Time time)
{
    return this->get_mutable_driving_scene_state(time);
}

IDrivingSceneState* ADrivingScene::get_mutable_driving_scene_state(temporal::Time time)
{
    return new BasicDrivingSceneState(this, time);
}

}
}
}
