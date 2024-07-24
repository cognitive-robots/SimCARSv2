#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/map/ped_map_interface.hpp>
#include <ori/simcars/map/thor_magni/thor_magni_node.hpp>

namespace ori
{
namespace simcars
{
namespace map
{
namespace thor_magni
{

class ThorMagniMap : public virtual IPedMap
{
    sf::Image image;
    sf::Texture texture;
    geometry::Vec texture_offset;

    uint64_t max_goal_id;

    structures::stl::STLDictionary<uint64_t, ThorMagniNode*> id_to_node_dict;

    bool is_node_within_dist(geometry::Vec const &position, FP_DATA_TYPE dist) const;
    structures::IArray<ThorMagniNode*>* get_nodes_within_dist(geometry::Vec const &position,
                                                              FP_DATA_TYPE dist) const;

public:
    ~ThorMagniMap() override;

    geometry::Vec get_map_centre() const override;
    geometry::Rect get_bounding_box() const override;
    FP_DATA_TYPE get_max_dim_size() const override;

    sf::Texture const& get_texture() const override;
    FP_DATA_TYPE get_texture_scale() const override;
    geometry::Vec const& get_texture_offset() const override;

    structures::IArray<INode const*>* get_nodes() const override;
    structures::IArray<INode const*>* get_goal_nodes() const override;
    structures::IArray<geometry::VecPair>* get_edges() const override;

    void load(std::string const &texture_file_path_str,
              std::string const &offset_json_file_path_str, std::string const &scene_file_path_str,
              std::string const &goals_file_path_str, FP_DATA_TYPE node_clearance = 0.75,
              FP_DATA_TYPE node_adjacency = 1.5, size_t random_sample_count = 2000);
};

}
}
}
}
