
#include <ori/simcars/causal/variable_types/endogenous/lane_transitions_calc.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

LaneTransitionsCalcVariable::LaneTransitionsCalcVariable(
        IEndogenousVariable<structures::stl::STLStackArray<uint64_t>> const *endogenous_parent,
        IVariable<structures::stl::STLStackArray<uint64_t>> const *other_parent,
        map::IMap const *map) :
    ABinaryEndogenousVariable(endogenous_parent, other_parent), map(map) {}

FP_DATA_TYPE LaneTransitionsCalcVariable::get_value() const
{
    // NOTE: This is currently setup in such a way as to only ever return 0, 1, or -1

    structures::stl::STLStackArray<uint64_t> prev_lanes = get_endogenous_parent()->get_value();
    structures::stl::STLStackArray<uint64_t> next_lanes = get_other_parent()->get_value();

    bool left_transition = false;
    bool right_transition = false;

    for (size_t i = 0; i < prev_lanes.count(); ++i)
    {
        map::ILane const *prev_lane = map->get_lane(prev_lanes[i]);
        map::ILane const *prev_left_adjacent_lane = prev_lane->get_left_adjacent_lane();
        map::ILane const *prev_right_adjacent_lane = prev_lane->get_right_adjacent_lane();

        for (size_t j = 0; j < next_lanes.count(); ++j)
        {
            if (next_lanes[j] == prev_left_adjacent_lane->get_id())
            {
                left_transition = true;
            }

            if (next_lanes[j] == prev_right_adjacent_lane->get_id())
            {
                right_transition = true;
            }

            if (left_transition && right_transition)
            {
                break;
            }
        }

        if (left_transition && right_transition)
        {
            break;
        }
    }

    if (left_transition)
    {
        if (right_transition)
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        if (right_transition)
        {
            return -1;
        }
        else
        {
            return 0;
        }
    }
}

}
}
}
