#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/map/laneletd/laneletd_declarations.hpp>
#include <ori/simcars/map/laneletd/laneletd_lane.hpp>

namespace ori
{
namespace simcars
{
namespace map
{
namespace laneletd
{

class LaneletDMap : public virtual IMap
{
    structures::stl::STLDictionary<uint64_t, LaneletDLane*> id_to_lane_dict;

public:
    ~LaneletDMap() override;

    ILane const* get_lane(uint64_t id) const override;
    structures::IArray<ILane const*>* get_lanes(structures::IArray<uint64_t> const *ids) const override;
    structures::IArray<ILane const*>* get_encapsulating_lanes(geometry::Vec point) const override;
    structures::IArray<ILane const*>* get_lanes_in_range(geometry::Vec point, FP_DATA_TYPE distance) const override;

    void save(std::string const &output_file_path_str, std::string const &recording_meta_file_path_str) const;

    void clear();
    void load(std::string const &input_file_path_str, std::string const &recording_meta_file_path_str);
};

}
}
}
}
