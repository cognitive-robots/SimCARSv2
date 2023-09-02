
#include <ori/simcars/causal/variable_types/endogenous/lane_encapsulation.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

LaneEncapsulationVariable::LaneEncapsulationVariable(
        IEndogenousVariable<uint64_t> const *endogenous_parent,
        IVariable<geometry::Vec> const *other_parent, map::IMap const *map) :
    ABinaryEndogenousVariable(endogenous_parent, other_parent), map(map)
{
    assert(map != nullptr);
}

bool LaneEncapsulationVariable::get_value() const
{
    map::ILane const *lane = map->get_lane(get_endogenous_parent()->get_value());
    if (lane != nullptr)
    {
        return lane->check_encapsulation(get_other_parent()->get_value());
    }
    else
    {
        return false;
    }
}

}
}
}
