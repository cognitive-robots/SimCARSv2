
#include <ori/simcars/causal/variable_types/endogenous/lane_map_point.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

LaneMapPointVariable::LaneMapPointVariable(
        IEndogenousVariable<uint64_t> *endogenous_parent,
        IVariable<geometry::Vec> *other_parent, map::IDrivingMap const *map) :
    ABinaryEndogenousVariable(endogenous_parent, other_parent), map(map) {}

bool LaneMapPointVariable::get_value(geometry::Vec &val) const
{
    uint64_t lane_id;
    if (get_endogenous_parent()->get_value(lane_id) && get_other_parent()->get_value(val))
    {
        map::ILane const *lane = map->get_lane(lane_id);
        if (lane != nullptr)
        {
            val = lane->map_point(val);
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool LaneMapPointVariable::set_value(geometry::Vec const &val)
{
    geometry::Vec map_point;
    if(get_value(map_point))
    {
        return val == map_point;
    }
    else
    {
        return true;
    }
}

}
}
}
