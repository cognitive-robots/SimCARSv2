
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/agent/driving_goal_extraction_agent.hpp>
#include <ori/simcars/agent/driving_goal_extraction_scene.hpp>

#include <tuple>
#include <cmath>
#include <iostream>

namespace ori
{
namespace simcars
{
namespace agent
{

DrivingGoalExtractionScene::~DrivingGoalExtractionScene()
{
    structures::IArray<IDrivingAgent const*> const *driving_agents = driving_agent_dict.get_values();

    for (size_t i = 0; i < driving_agents->count(); ++i)
    {
        delete (*driving_agents)[i];
    }
}

DrivingGoalExtractionScene const* DrivingGoalExtractionScene::construct_from(IDrivingScene const *driving_scene)
{
    DrivingGoalExtractionScene *new_driving_scene = new DrivingGoalExtractionScene;

    new_driving_scene->min_spatial_limits = driving_scene->get_min_spatial_limits();
    new_driving_scene->max_spatial_limits = driving_scene->get_max_spatial_limits();
    new_driving_scene->min_temporal_limit = driving_scene->get_min_temporal_limit();
    new_driving_scene->max_temporal_limit = driving_scene->get_max_temporal_limit();

    structures::IArray<IDrivingAgent const*> *driving_agents =
            driving_scene->get_driving_agents();

    for(size_t i = 0; i < driving_agents->count(); ++i)
    {
        IDrivingAgent *driving_goal_extraction_agent = new DrivingGoalExtractionAgent((*driving_agents)[i]);

        new_driving_scene->driving_agent_dict.update(driving_goal_extraction_agent->get_name(), driving_goal_extraction_agent);
    }

    delete driving_agents;

    return new_driving_scene;
}

DrivingGoalExtractionScene const* DrivingGoalExtractionScene::construct_from(IDrivingScene const *driving_scene,
                                                                             map::IMap<std::string> const *map)
{
    DrivingGoalExtractionScene *new_driving_scene = new DrivingGoalExtractionScene;

    new_driving_scene->min_spatial_limits = driving_scene->get_min_spatial_limits();
    new_driving_scene->max_spatial_limits = driving_scene->get_max_spatial_limits();
    new_driving_scene->min_temporal_limit = driving_scene->get_min_temporal_limit();
    new_driving_scene->max_temporal_limit = driving_scene->get_max_temporal_limit();

    new_driving_scene->map = map;

    structures::IArray<IDrivingAgent const*> *driving_agents =
            driving_scene->get_driving_agents();

    for(size_t i = 0; i < driving_agents->count(); ++i)
    {
        IDrivingAgent *driving_goal_extraction_agent = new DrivingGoalExtractionAgent((*driving_agents)[i], map);

        new_driving_scene->driving_agent_dict.update(driving_goal_extraction_agent->get_name(), driving_goal_extraction_agent);
    }

    delete driving_agents;

    return new_driving_scene;
}

geometry::Vec DrivingGoalExtractionScene::get_min_spatial_limits() const
{
    return min_spatial_limits;
}

geometry::Vec DrivingGoalExtractionScene::get_max_spatial_limits() const
{
    return max_spatial_limits;
}

temporal::Time DrivingGoalExtractionScene::get_min_temporal_limit() const
{
    return min_temporal_limit;
}

temporal::Time DrivingGoalExtractionScene::get_max_temporal_limit() const
{
    return max_temporal_limit;
}

structures::IArray<IEntity const*>* DrivingGoalExtractionScene::get_entities() const
{
    structures::IArray<IDrivingAgent const*>* const driving_agents = this->get_driving_agents();
    structures::IArray<IEntity const*>* const entities =
                new structures::stl::STLStackArray<IEntity const*>(driving_agents->count());
    cast_array(*driving_agents, *entities);
    delete driving_agents;
    return entities;
}

IEntity const* DrivingGoalExtractionScene::get_entity(const std::string& entity_name) const
{
    return this->get_driving_agent(entity_name);
}

structures::IArray<IDrivingAgent const*>* DrivingGoalExtractionScene::get_driving_agents() const
{
    structures::stl::STLStackArray<IDrivingAgent const*> *driving_agents =
            new structures::stl::STLStackArray<IDrivingAgent const*>(driving_agent_dict.count());
    driving_agent_dict.get_values(driving_agents);
    return driving_agents;
}

IDrivingAgent const* DrivingGoalExtractionScene::get_driving_agent(const std::string& driving_agent_name) const
{
    return driving_agent_dict[driving_agent_name];
}

bool DrivingGoalExtractionScene::has_map() const
{
    return map != nullptr;
}

map::IMap<std::string> const* DrivingGoalExtractionScene::get_map() const
{
    return map;
}

}
}
}
