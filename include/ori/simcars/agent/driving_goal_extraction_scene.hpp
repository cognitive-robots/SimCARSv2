#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/map/map_interface.hpp>
#include <ori/simcars/agent/driving_scene_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class DrivingGoalExtractionScene : public virtual ADrivingScene
{
    geometry::Vec min_spatial_limits, max_spatial_limits;
    temporal::Time min_temporal_limit, max_temporal_limit;

    structures::stl::STLDictionary<std::string, IDrivingAgent const*> driving_agent_dict;

    map::IMap<std::string> const *map;

protected:
    DrivingGoalExtractionScene() : map(nullptr) {}

public:
    ~DrivingGoalExtractionScene();

    static DrivingGoalExtractionScene const* construct_from(IDrivingScene const *driving_scene);
    static DrivingGoalExtractionScene const* construct_from(IDrivingScene const *driving_scene,
                                                            map::IMap<std::string> const *map);

    geometry::Vec get_min_spatial_limits() const override;
    geometry::Vec get_max_spatial_limits() const override;

    temporal::Time get_min_temporal_limit() const override;
    temporal::Time get_max_temporal_limit() const override;

    structures::IArray<IEntity const*>* get_entities() const override;
    IEntity const* get_entity(std::string const &entity_name) const override;

    structures::IArray<IDrivingAgent const*>* get_driving_agents() const override;
    IDrivingAgent const* get_driving_agent(std::string const &driving_agent_name) const override;

    bool has_map() const;
    map::IMap<std::string> const* get_map() const;
};

}
}
}
