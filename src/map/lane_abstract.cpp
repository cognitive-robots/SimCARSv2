
#include <ori/simcars/map/lane_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

ALane::ALane(uint64_t id, uint64_t left_adjacent_lane_id, uint64_t right_adjacent_lane_id,
             structures::IArray<uint64_t> *fore_lane_ids,
             structures::IArray<uint64_t> *aft_lane_ids, IDrivingMap const *map) :
    ADrivingMapObject(id, map), left_adjacent_lane_id(left_adjacent_lane_id),
    right_adjacent_lane_id(right_adjacent_lane_id), left_adjacent_lane(nullptr),
    right_adjacent_lane(nullptr), fore_lane_branch(fore_lane_ids, map),
    aft_lane_branch(aft_lane_ids, map) {}

ALane::Type ALane::get_type() const
{
    return ALane::Type::LANE;
}

ILane const* ALane::get_left_adjacent_lane() const
{
    if (left_adjacent_lane == nullptr && left_adjacent_lane_id > 0)
    {
        left_adjacent_lane = get_map()->get_lane(left_adjacent_lane_id);
    }

    return left_adjacent_lane;
}

ILane const* ALane::get_right_adjacent_lane() const
{
    if (right_adjacent_lane == nullptr && right_adjacent_lane_id > 0)
    {
        right_adjacent_lane = get_map()->get_lane(right_adjacent_lane_id);
    }

    return right_adjacent_lane;
}

LaneBranch const* ALane::get_fore_lane_branch() const
{
    return &fore_lane_branch;
}

LaneBranch const* ALane::get_aft_lane_branch() const
{
    return &aft_lane_branch;
}

}
}
}
