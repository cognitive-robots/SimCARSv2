
#include <ori/simcars/causal/variable_types/endogenous/lane_selectable.hpp>

#include <ori/simcars/structures/stl/stl_set.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

LaneSelectableVariable::LaneSelectableVariable(IVariable<geometry::Vec> const *parent,
                                       map::IMap const *map) :
    AUnaryEndogenousVariable(parent), map(map)
{
    assert(map != nullptr);
}

structures::stl::STLStackArray<uint64_t> LaneSelectableVariable::get_value() const
{
    structures::IArray<map::ILane const*> *encapsulating_lanes =
            map->get_encapsulating_lanes(get_parent()->get_value());

    structures::stl::STLSet<uint64_t> selected_lanes;

    size_t i;

    for (i = 0; i < encapsulating_lanes->count(); ++i)
    {
        map::ILane const *encapsulating_lane = (*encapsulating_lanes)[i];

        selected_lanes.insert(encapsulating_lane->get_id());

        map::ILane const *left_lane = encapsulating_lane->get_left_adjacent_lane();

        if (left_lane != nullptr)
        {
            selected_lanes.insert(left_lane->get_id());
        }

        map::ILane const *right_lane = encapsulating_lane->get_right_adjacent_lane();

        if (right_lane != nullptr)
        {
            selected_lanes.insert(right_lane->get_id());
        }

        map::LaneBranch const *fore_lane_branch = encapsulating_lane->get_fore_lane_branch();

        size_t j;
        map::ILane const *fore_lane;

        for (j = 0; (fore_lane = (*fore_lane_branch)[j]) != nullptr; ++i)
        {
            selected_lanes.insert(fore_lane->get_id());

            left_lane = fore_lane->get_left_adjacent_lane();

            if (left_lane != nullptr)
            {
                selected_lanes.insert(left_lane->get_id());
            }

            right_lane = fore_lane->get_right_adjacent_lane();

            if (right_lane != nullptr)
            {
                selected_lanes.insert(right_lane->get_id());
            }
        }

        for (j = -1; (fore_lane = (*fore_lane_branch)[j]) != nullptr; --i)
        {
            selected_lanes.insert(fore_lane->get_id());

            left_lane = fore_lane->get_left_adjacent_lane();

            if (left_lane != nullptr)
            {
                selected_lanes.insert(left_lane->get_id());
            }

            right_lane = fore_lane->get_right_adjacent_lane();

            if (right_lane != nullptr)
            {
                selected_lanes.insert(right_lane->get_id());
            }
        }
    }

    delete encapsulating_lanes;

    return selected_lanes.get_array();
}

}
}
}
