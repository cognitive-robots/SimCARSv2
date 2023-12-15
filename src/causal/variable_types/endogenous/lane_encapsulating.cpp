
#include <ori/simcars/causal/variable_types/endogenous/lane_encapsulating.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

LaneEncapsulatingVariable::LaneEncapsulatingVariable(IVariable<geometry::Vec> const *parent,
                                       map::IMap const *map) :
    AUnaryEndogenousVariable(parent), map(map)
{
    assert(map != nullptr);
}

structures::stl::STLStackArray<uint64_t> LaneEncapsulatingVariable::get_value() const
{
    structures::IArray<map::ILane const*> *encapsulating_lanes =
            map->get_encapsulating_lanes(get_parent()->get_value());

    structures::stl::STLStackArray<uint64_t> selected_lanes;

    size_t i;

    for (i = 0; i < encapsulating_lanes->count(); ++i)
    {
        map::ILane const *encapsulating_lane = (*encapsulating_lanes)[i];

        selected_lanes.push_back(encapsulating_lane->get_id());
    }

    delete encapsulating_lanes;

    return selected_lanes;
}

}
}
}
