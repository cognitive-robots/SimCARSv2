
#include <ori/simcars/causal/variable_types/endogenous/lane_encapsulation.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

LaneEncapsulationVariable::LaneEncapsulationVariable(
        IEndogenousVariable<uint64_t> *endogenous_parent,
        IVariable<geometry::Vec> *other_parent, map::IMap const *map) :
    ABinaryEndogenousVariable(endogenous_parent, other_parent), map(map)
{
    assert(map != nullptr);
}

bool LaneEncapsulationVariable::get_value(bool &val) const
{
    uint64_t lane_id;
    geometry::Vec pos;
    if (get_endogenous_parent()->get_value(lane_id) && get_other_parent()->get_value(pos))
    {
        map::ILane const *lane = map->get_lane(lane_id);
        if (lane != nullptr)
        {
            val = lane->check_encapsulation(pos);
        }
        else
        {
            val = false;
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool LaneEncapsulationVariable::set_value(bool const &val)
{
    bool encapsulation;
    if (get_value(encapsulation))
    {
        return val == encapsulation;
    }
    else
    {
        return true;
    }
}

}
}
}
