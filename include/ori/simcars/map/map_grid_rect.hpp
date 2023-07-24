#pragma once

#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/grid_rect.hpp>
#include <ori/simcars/map/lane_interface.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

class MapGridRect : public geometry::GridRect<MapGridRect>
{
    structures::stl::STLSet<ILane const*> lanes;

public:
    MapGridRect(geometry::Vec origin, FP_DATA_TYPE size);

    structures::ISet<ILane const*> const* get_lanes() const;
    structures::IArray<ILane const*>* get_encapsulating_lanes(geometry::Vec point) const;

    void insert_lane(ILane const *lane);
    void erase_lane(ILane const *lane);
};

}
}
}
