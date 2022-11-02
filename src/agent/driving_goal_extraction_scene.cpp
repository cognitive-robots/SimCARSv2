
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

std::shared_ptr<const DrivingGoalExtractionScene> DrivingGoalExtractionScene::construct_from(std::shared_ptr<const IDrivingScene> driving_scene)
{
    std::shared_ptr<DrivingGoalExtractionScene> new_driving_scene(new DrivingGoalExtractionScene());

    new_driving_scene->min_spatial_limits = driving_scene->get_min_spatial_limits();
    new_driving_scene->max_spatial_limits = driving_scene->get_max_spatial_limits();
    new_driving_scene->min_temporal_limit = driving_scene->get_min_temporal_limit();
    new_driving_scene->max_temporal_limit = driving_scene->get_max_temporal_limit();

    std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgent>>> driving_agents =
            driving_scene->get_driving_agents();

    for(size_t i = 0; i < driving_agents->count(); ++i)
    {
        std::shared_ptr<IDrivingAgent> driving_goal_extraction_agent =
                std::shared_ptr<IDrivingAgent>(new DrivingGoalExtractionAgent((*driving_agents)[i]));

        new_driving_scene->driving_agent_dict.update(driving_goal_extraction_agent->get_name(), driving_goal_extraction_agent);
    }

    return new_driving_scene;
}

std::shared_ptr<const DrivingGoalExtractionScene> DrivingGoalExtractionScene::construct_from(std::shared_ptr<const IDrivingScene> driving_scene,
                                                                           std::shared_ptr<const map::IMap<std::string>> map)
{
    std::shared_ptr<DrivingGoalExtractionScene> new_driving_scene(new DrivingGoalExtractionScene());

    new_driving_scene->min_spatial_limits = driving_scene->get_min_spatial_limits();
    new_driving_scene->max_spatial_limits = driving_scene->get_max_spatial_limits();
    new_driving_scene->min_temporal_limit = driving_scene->get_min_temporal_limit();
    new_driving_scene->max_temporal_limit = driving_scene->get_max_temporal_limit();

    new_driving_scene->map = map;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgent>>> driving_agents =
            driving_scene->get_driving_agents();

    for(size_t i = 0; i < driving_agents->count(); ++i)
    {
        std::shared_ptr<IDrivingAgent> driving_goal_extraction_agent =
                std::shared_ptr<IDrivingAgent>(new DrivingGoalExtractionAgent((*driving_agents)[i], map));

        new_driving_scene->driving_agent_dict.update(driving_goal_extraction_agent->get_name(), driving_goal_extraction_agent);
    }

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

std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> DrivingGoalExtractionScene::get_entities() const
{
    const std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgent>>> driving_agents = this->get_driving_agents();
    const std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> entities(
                new structures::stl::STLStackArray<std::shared_ptr<const IEntity>>(driving_agents->count()));
    cast_array(*driving_agents, *entities);
    return entities;
}

std::shared_ptr<const IEntity> DrivingGoalExtractionScene::get_entity(const std::string& entity_name) const
{
    return this->get_driving_agent(entity_name);
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgent>>> DrivingGoalExtractionScene::get_driving_agents() const
{
    return std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgent>>>(
            new structures::stl::STLStackArray<std::shared_ptr<const IDrivingAgent>>(driving_agent_dict.get_values()));
}

std::shared_ptr<const IDrivingAgent> DrivingGoalExtractionScene::get_driving_agent(const std::string& driving_agent_name) const
{
    return driving_agent_dict[driving_agent_name];
}

bool DrivingGoalExtractionScene::has_map() const
{
    return map != nullptr;
}

std::shared_ptr<const map::IMap<std::string>> DrivingGoalExtractionScene::get_map() const
{
    return map;
}

}
}
}
