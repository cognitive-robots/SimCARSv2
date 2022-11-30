#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agent/file_based_scene_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{
namespace csv
{

class CSVScene : public virtual AFileBasedScene<CSVScene>
{
    geometry::Vec min_spatial_limits, max_spatial_limits;
    temporal::Time min_temporal_limit, max_temporal_limit;

    structures::stl::STLDictionary<std::string, IEntity const*> entity_dict;

protected:
    void save_virt(std::ofstream &output_filestream) const override;
    void load_virt(std::ifstream &input_filestream, structures::ISet<std::string>* agent_names) override;

public:
    static CSVScene const* construct_from(IScene const *scene);

    geometry::Vec get_min_spatial_limits() const override;
    geometry::Vec get_max_spatial_limits() const override;

    temporal::Time get_min_temporal_limit() const override;
    temporal::Time get_max_temporal_limit() const override;

    structures::IArray<IEntity const*>* get_entities() const override;
    IEntity const* get_entity(std::string const &entity_name) const override;

    ISceneState* get_state(temporal::Time time) const override;
};

}
}
}
}
