#pragma once

#include <ori/simcars/structures/set_interface.hpp>
#include <ori/simcars/structures/dictionary_interface.hpp>
#include <ori/simcars/map/file_based_map_abstract.hpp>
#include <ori/simcars/map/highd/highd_declarations.hpp>

namespace ori
{
namespace simcars
{
namespace map
{
namespace highd
{

class HighDMap : public virtual AFileBasedMap<uint8_t, HighDMap>
{
    structures::IDictionary<uint8_t, HighDLane*> *id_to_lane_dict;

    structures::ISet<IMapObject<uint8_t> const*> *stray_ghosts;

protected:
    void save_virt(std::ofstream &output_filestream) const override;
    void load_virt(std::ifstream &input_filestream) override;

public:
    ~HighDMap() override;

    ILane<uint8_t> const* get_lane(uint8_t id) const override;
    ILaneArray<uint8_t> const* get_encapsulating_lanes(geometry::Vec point) const override;
    ILaneArray<uint8_t> const* get_lanes(structures::IArray<uint8_t> const *ids) const override;
    ILaneArray<uint8_t> const* get_lanes_in_range(geometry::Vec point, FP_DATA_TYPE distance) const override;
    ITrafficLight<uint8_t> const* get_traffic_light(uint8_t id) const override;
    ITrafficLightArray<uint8_t> const* get_traffic_lights(structures::IArray<uint8_t> const *ids) const override;
    ITrafficLightArray<uint8_t> const* get_traffic_lights_in_range(geometry::Vec point, FP_DATA_TYPE distance) const override;
    void register_stray_ghost(IMapObject<uint8_t> const *ghost) const override;
    void unregister_stray_ghost(IMapObject<uint8_t> const *ghost) const override;

    HighDMap* shallow_copy() const override;
};

}
}
}
}
