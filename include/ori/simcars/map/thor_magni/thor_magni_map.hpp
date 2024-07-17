#pragma once

#include <ori/simcars/map/ped_map_interface.hpp>

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
    sf::Texture texture;
    geometry::Vec texture_offset;

public:
    geometry::Vec get_map_centre() const override;
    FP_DATA_TYPE get_max_dim_size() const override;
    sf::Sprite* get_sprite() const override;

    void load(std::string const &texture_file_path_str,
              std::string const &offset_json_file_path_str,
              std::string const &scene_file_path_str);
};

}
}
}
}
