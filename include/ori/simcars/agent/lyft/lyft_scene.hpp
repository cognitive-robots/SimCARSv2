#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agent/driving_scene_interface.hpp>
#include <ori/simcars/agent/file_based_scene_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{
namespace lyft
{

class LyftScene : public virtual AFileBasedScene<LyftScene>, public virtual IDrivingScene
{
    geometry::Vec min_spatial_limits, max_spatial_limits;
    temporal::Time min_temporal_limit, max_temporal_limit;

    structures::stl::STLDictionary<std::string, std::shared_ptr<const IDrivingAgent>> driving_agent_dict;

protected:
    void save_virt(std::ofstream& output_filestream) const override;
    void load_virt(std::ifstream& input_filestream) override;

public:
    static std::shared_ptr<const LyftScene> construct_from(std::shared_ptr<const IDrivingScene> driving_scene);

    geometry::Vec get_min_spatial_limits() const override;
    geometry::Vec get_max_spatial_limits() const override;

    temporal::Time get_min_temporal_limit() const override;
    temporal::Time get_max_temporal_limit() const override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> get_entities() const override;
    std::shared_ptr<const IEntity> get_entity(const std::string& entity_name) const override;

    std::shared_ptr<const IState> get_state(temporal::Time time) const override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgent>>> get_driving_agents() const override;
    std::shared_ptr<const IDrivingAgent> get_driving_agent(const std::string& driving_agent_name) const override;

    std::shared_ptr<const IDrivingSceneState> get_driving_scene_state(temporal::Time time) const override;
};

}
}
}
}
