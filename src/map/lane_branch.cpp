
#include <ori/simcars/map/lane_branch.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

void LaneBranch::retrieve_lanes() const
{
    structures::IArray<ILane const*> const *lanes = map->get_lanes(lane_ids);
    delete lane_ids;

    if (lanes->count() > 0)
    {
        straightest_lane = (*lanes)[0];

        size_t i, j;
        for (i = 1; i < lanes->count(); ++i)
        {
            ILane const *current_lane = (*lanes)[i];

            if (std::abs(current_lane->get_curvature()) <
                    std::abs(straightest_lane->get_curvature()))
            {
                if (current_lane->get_curvature() > 0)
                {
                    left_curving_lanes.push_front(straightest_lane);
                }
                else
                {
                    right_curving_lanes.push_front(straightest_lane);
                }

                straightest_lane = current_lane;
            }
            else
            {
                structures::stl::STLDequeArray<ILane const*> *curving_lanes;

                if (current_lane->get_curvature() > 0)
                {
                    curving_lanes = &left_curving_lanes;
                }
                else
                {
                    curving_lanes = &right_curving_lanes;
                }

                for (j = 0; j < curving_lanes->count(); ++j)
                {
                    ILane const *current_curving_lane = (*curving_lanes)[i];

                    if (std::abs(current_lane->get_curvature()) <
                            std::abs(current_curving_lane->get_curvature()))
                    {
                        break;
                    }
                }

                curving_lanes->push_at(j, current_lane);
            }
        }
    }

    delete lanes;
}

LaneBranch::LaneBranch(structures::IArray<uint64_t> *lane_ids, IMap const *map) :
    lane_ids(lane_ids), map(map), straightest_lane(nullptr)
{
    assert(lane_ids != nullptr);
    assert(map != nullptr);
}

LaneBranch::~LaneBranch()
{
    if (lane_ids != nullptr) delete lane_ids;
}

size_t LaneBranch::count() const
{
    return lane_ids->count();
}

bool LaneBranch::contains(ILane const* const &val) const
{
    return lane_ids->contains(val->get_id());
}

ILane const* LaneBranch::operator [](size_t idx) const
{
    if (idx < 0)
    {
        if (1 - idx >= left_curving_lanes.count())
        {
            return nullptr;
        }
        else
        {
            return left_curving_lanes[1 - idx];
        }
    }
    else if (idx > 0)
    {
        if (idx - 1 >= right_curving_lanes.count())
        {
            return nullptr;
        }
        else
        {
            return right_curving_lanes[idx - 1];
        }
    }
    else
    {
        return straightest_lane;
    }
}

}
}
}
