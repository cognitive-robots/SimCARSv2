
#include <ori/simcars/map/lane_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

ALane::ALane(uint64_t id, uint64_t left_adjacent_lane_id, uint64_t right_adjacent_lane_id,
             structures::IArray<uint64_t> *fore_lane_ids,
             structures::IArray<uint64_t> *aft_lane_ids, IMap const *map) : AMapObject(id, map),
    left_adjacent_lane_id(left_adjacent_lane_id), right_adjacent_lane_id(right_adjacent_lane_id),
    fore_lane_ids(fore_lane_ids), aft_lane_ids(aft_lane_ids),
    left_adjacent_lane(nullptr), right_adjacent_lane(nullptr),
    straightest_fore_lane(nullptr), fore_lanes(nullptr), aft_lanes(nullptr)
{
    assert(fore_lane_ids != nullptr);
    assert(aft_lane_ids != nullptr);
}

ALane::~ALane()
{
    delete fore_lane_ids;
    delete aft_lane_ids;
    if (fore_lanes != nullptr) delete fore_lanes;
    if (aft_lanes != nullptr) delete aft_lanes;
}

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

ILane const* ALane::get_straightest_fore_lane() const
{
    if (straightest_fore_lane == nullptr)
    {
        structures::IArray<ILane const*> const *fore_lanes = get_fore_lanes();
        if (fore_lanes->count() > 0)
        {
            ILane const *current_straightest_fore_lane = (*fore_lanes)[0];
            for (size_t i = 1; i < fore_lanes->count(); ++i)
            {
                if (std::abs((*fore_lanes)[i]->get_curvature()) <
                        std::abs(current_straightest_fore_lane->get_curvature()))
                {
                    current_straightest_fore_lane = (*fore_lanes)[i];
                }
            }
            straightest_fore_lane = current_straightest_fore_lane;
        }
    }

    return straightest_fore_lane;
}

structures::IArray<ILane const*> const* ALane::get_fore_lanes() const
{
    if (fore_lanes == nullptr)
    {
        fore_lanes = get_map()->get_lanes(fore_lane_ids);
    }

    return fore_lanes;
}

structures::IArray<ILane const*> const* ALane::get_aft_lanes() const
{
    if (aft_lanes == nullptr)
    {
        aft_lanes = get_map()->get_lanes(aft_lane_ids);
    }

    return aft_lanes;
}

}
}
}
