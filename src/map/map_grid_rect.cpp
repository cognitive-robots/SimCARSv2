
#include <ori/simcars/map/map_grid_rect.hpp>

#include <ori/simcars/structures/stl/stl_stack_array.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

MapGridRect::MapGridRect(geometry::Vec origin, FP_DATA_TYPE size)
    : geometry::GridRect<MapGridRect>(origin, size, size) {}

structures::ISet<ILane const*> const* MapGridRect::get_lanes() const
{
    return &lanes;
}
structures::IArray<ILane const*>* MapGridRect::get_encapsulating_lanes(geometry::Vec point) const
{
    structures::IArray<ILane const*> const *lane_array = lanes.get_array();
    structures::IStackArray<ILane const*> *encapsulating_lanes =
            new structures::stl::STLStackArray<ILane const*>;

    for (size_t i = 0; i < lane_array->count(); ++i)
    {
        if ((*lane_array)[i]->check_encapsulation(point))
        {
            encapsulating_lanes->push_back((*lane_array)[i]);
        }
    }

    return encapsulating_lanes;
}

void MapGridRect::insert_lane(ILane const *lane)
{
    lanes.insert(lane);
}
void MapGridRect::erase_lane(ILane const *lane)
{
    lanes.erase(lane);
}

}
}
}
