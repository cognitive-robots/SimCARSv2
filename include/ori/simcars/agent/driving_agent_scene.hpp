#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/map/map_interface.hpp>
#include <ori/simcars/agent/scene_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class DrivingAgentScene : public virtual AScene
{
    geometry::Vec min_spatial_limits, max_spatial_limits;
    temporal::Time min_temporal_limit, max_temporal_limit;

    structures::stl::STLDictionary<std::string, std::shared_ptr<const IEntity>> entity_dict;

    std::shared_ptr<const map::IMap<std::string>> map = nullptr;

    void extract_speed_change_events(std::shared_ptr<IEntity> entity);
    void extract_lane_change_events(std::shared_ptr<IEntity> entity, std::shared_ptr<const map::IMap<std::string>> map);

public:
    static std::shared_ptr<const DrivingAgentScene> construct_from(std::shared_ptr<const IScene> scene);
    static std::shared_ptr<const DrivingAgentScene> construct_from(std::shared_ptr<const IScene> scene,
                                                                   std::shared_ptr<const map::IMap<std::string>> map);

    geometry::Vec get_min_spatial_limits() const override;
    geometry::Vec get_max_spatial_limits() const override;

    temporal::Time get_min_temporal_limit() const override;
    temporal::Time get_max_temporal_limit() const override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> get_entities() const override;
    std::shared_ptr<const IEntity> get_entity(const std::string& entity_name) const override;

    bool has_map() const;
    std::shared_ptr<const map::IMap<std::string>> get_map() const;
};

}
}
}
