#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/geometry/grid_dictionary.hpp>
#include <ori/simcars/map/file_based_map_abstract.hpp>
#include <ori/simcars/map/weak_living_lane_stack_array.hpp>
#include <ori/simcars/map/weak_living_traffic_light_stack_array.hpp>
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
    std::shared_ptr<structures::stl::STLDictionary<std::string, std::shared_ptr<LyftLane>>> id_to_lane_dict;
    std::shared_ptr<structures::stl::STLDictionary<std::string, std::shared_ptr<LyftTrafficLight>>> id_to_traffic_light_dict;
    std::shared_ptr<geometry::GridDictionary<MapGridRect<std::string>>> map_grid_dict;

protected:
    void save_virt(std::ofstream& output_filestream) const override;
    void load_virt(std::ifstream& input_filestream) override;

public:
    std::shared_ptr<const ILane<std::string>> get_lane(std::string id) const override;
    std::shared_ptr<const ILaneArray<std::string>> get_encapsulating_lanes(geometry::Vec point) const override;
    std::shared_ptr<const ILaneArray<std::string>> get_lanes(std::shared_ptr<const structures::IArray<std::string>> ids) const override;
    std::shared_ptr<const ILaneArray<std::string>> get_lanes_in_range(geometry::Vec point, FP_DATA_TYPE distance) const override;
    std::shared_ptr<const ITrafficLight<std::string>> get_traffic_light(std::string id) const override;
    std::shared_ptr<const ITrafficLightArray<std::string>> get_traffic_lights(std::shared_ptr<const structures::IArray<std::string>> ids) const override;
    std::shared_ptr<const ITrafficLightArray<std::string>> get_traffic_lights_in_range(geometry::Vec point, FP_DATA_TYPE distance) const override;

    std::shared_ptr<LyftMap> copy() const override;
};

}
}
}
}
