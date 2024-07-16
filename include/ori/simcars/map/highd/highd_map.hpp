#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/map/highd/highd_declarations.hpp>
#include <ori/simcars/map/highd/highd_lane.hpp>

namespace ori
{
namespace simcars
{
namespace map
{
namespace highd
{

class HighDMap : public virtual IDrivingMap
{
    structures::stl::STLDictionary<uint64_t, HighDLane*> id_to_lane_dict;

public:
    ~HighDMap() override;

    geometry::Vec get_map_centre() const override;
    FP_DATA_TYPE get_max_dim_size() const override;
    ILane const* get_lane(uint64_t id) const override;
    structures::IArray<ILane const*>* get_lanes(structures::IArray<uint64_t> const *ids) const override;
    structures::IArray<ILane const*>* get_encapsulating_lanes(geometry::Vec point) const override;
    structures::IArray<ILane const*>* get_lanes_in_range(geometry::Vec point, FP_DATA_TYPE distance) const override;

    void save(std::string const &output_file_path_str) const;

    void clear();
    void load(std::string const &input_file_path_str);
};

}
}
}
}
