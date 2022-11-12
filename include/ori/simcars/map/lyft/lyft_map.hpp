#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/geometry/grid_dictionary.hpp>
#include <ori/simcars/map/file_based_map_abstract.hpp>
#include <ori/simcars/map/living_lane_stack_array.hpp>
#include <ori/simcars/map/living_traffic_light_stack_array.hpp>
#include <ori/simcars/map/map_grid_rect.hpp>
#include <ori/simcars/map/lyft/lyft_declarations.hpp>
#include <ori/simcars/map/lyft/lyft_lane.hpp>
#include <ori/simcars/map/lyft/lyft_traffic_light.hpp>

#include <string>

namespace ori
{
namespace simcars
{
namespace map
{
namespace lyft
{

class LyftMap : public virtual AFileBasedMap<std::string, LyftMap>
{
    structures::IDictionary<std::string, LyftLane*> *id_to_lane_dict;
    structures::IDictionary<std::string, LyftTrafficLight*> *id_to_traffic_light_dict;
    geometry::GridDictionary<MapGridRect<std::string>> *map_grid_dict;

protected:
    void save_virt(std::ofstream &output_filestream) const override;
    void load_virt(std::ifstream &input_filestream) override;

public:
    ILane<std::string> const* get_lane(std::string id) const override;
    ILaneArray<std::string> const* get_encapsulating_lanes(geometry::Vec point) const override;
    ILaneArray<std::string> const* get_lanes(structures::IArray<std::string> const *ids) const override;
    ILaneArray<std::string> const* get_lanes_in_range(geometry::Vec point, FP_DATA_TYPE distance) const override;
    ITrafficLight<std::string> const* get_traffic_light(std::string id) const override;
    ITrafficLightArray<std::string> const* get_traffic_lights(structures::IArray<std::string> const *ids) const override;
    ITrafficLightArray<std::string> const* get_traffic_lights_in_range(geometry::Vec point, FP_DATA_TYPE distance) const override;

    LyftMap* copy() const override;
};

}
}
}
}
