#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/map/map_interface.hpp>
#include <ori/simcars/agent/driving_scene_abstract.hpp>
#include <ori/simcars/agent/driving_goal_extraction_agent.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T_map_id>
class DrivingGoalExtractionScene : public virtual ADrivingScene
{
    geometry::Vec min_spatial_limits, max_spatial_limits;
    temporal::Time min_temporal_limit, max_temporal_limit;

    structures::stl::STLDictionary<std::string, IDrivingAgent const*> driving_agent_dict;

    map::IMap<T_map_id> const *map;

protected:
    DrivingGoalExtractionScene() : map(nullptr) {}

public:
    ~DrivingGoalExtractionScene()
    {
        structures::IArray<IDrivingAgent const*> const *driving_agents = driving_agent_dict.get_values();

        for (size_t i = 0; i < driving_agents->count(); ++i)
        {
            delete (*driving_agents)[i];
        }
    }

    static DrivingGoalExtractionScene const* construct_from(IDrivingScene const *driving_scene)
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
            IDrivingAgent *driving_goal_extraction_agent = new DrivingGoalExtractionAgent<T_map_id>((*driving_agents)[i]);

            new_driving_scene->driving_agent_dict.update(driving_goal_extraction_agent->get_name(), driving_goal_extraction_agent);
        }

        delete driving_agents;

        return new_driving_scene;
    }
    static DrivingGoalExtractionScene const* construct_from(IDrivingScene const *driving_scene,
                                                            map::IMap<T_map_id> const *map)
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
            IDrivingAgent *driving_goal_extraction_agent = new DrivingGoalExtractionAgent<T_map_id>((*driving_agents)[i], map);

            new_driving_scene->driving_agent_dict.update(driving_goal_extraction_agent->get_name(), driving_goal_extraction_agent);
        }

        delete driving_agents;

        return new_driving_scene;
    }

    geometry::Vec get_min_spatial_limits() const override
    {
        return min_spatial_limits;
    }
    geometry::Vec get_max_spatial_limits() const override
    {
        return max_spatial_limits;
    }

    temporal::Time get_min_temporal_limit() const override
    {
        return min_temporal_limit;
    }
    temporal::Time get_max_temporal_limit() const override
    {
        return max_temporal_limit;
    }

    structures::IArray<IEntity const*>* get_entities() const override
    {
        structures::IArray<IDrivingAgent const*>* const driving_agents = this->get_driving_agents();
        structures::IArray<IEntity const*>* const entities =
                    new structures::stl::STLStackArray<IEntity const*>(driving_agents->count());
        cast_array(*driving_agents, *entities);
        delete driving_agents;
        return entities;
    }
    IEntity const* get_entity(std::string const &entity_name) const override
    {
        return this->get_driving_agent(entity_name);
    }

    structures::IArray<IDrivingAgent const*>* get_driving_agents() const override
    {
        structures::stl::STLStackArray<IDrivingAgent const*> *driving_agents =
                new structures::stl::STLStackArray<IDrivingAgent const*>(driving_agent_dict.count());
        driving_agent_dict.get_values(driving_agents);
        return driving_agents;
    }
    IDrivingAgent const* get_driving_agent(std::string const &driving_agent_name) const override
    {
        return driving_agent_dict[driving_agent_name];
    }

    bool has_map() const
    {
        return map != nullptr;
    }
    map::IMap<T_map_id> const* get_map() const
    {
        return map;
    }
};

}
}
}
