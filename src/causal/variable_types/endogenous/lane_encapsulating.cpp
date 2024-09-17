
#include <ori/simcars/causal/variable_types/endogenous/lane_encapsulating.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

LaneEncapsulatingVariable::LaneEncapsulatingVariable(IVariable<geometry::Vec> *parent,
                                       map::IDrivingMap const *map) :
    AUnaryEndogenousVariable(parent), map(map)
{
    assert(map != nullptr);
}

bool LaneEncapsulatingVariable::get_value(structures::stl::STLStackArray<uint64_t> &val) const
{
    geometry::Vec pos;
    if (get_parent()->get_value(pos))
    {
        structures::IArray<map::ILane const*> *encapsulating_lanes =
                map->get_encapsulating_lanes(pos);

        size_t i;

        for (i = 0; i < encapsulating_lanes->count(); ++i)
        {
            map::ILane const *encapsulating_lane = (*encapsulating_lanes)[i];

            val.push_back(encapsulating_lane->get_id());
        }

        delete encapsulating_lanes;

        return true;
    }
    else
    {
        return false;
    }
}

bool LaneEncapsulatingVariable::set_value(structures::stl::STLStackArray<uint64_t> const &val)
{
    geometry::Vec pos;
    if (get_parent()->get_value(pos))
    {
        structures::IArray<map::ILane const*> *encapsulating_lanes =
                map->get_encapsulating_lanes(pos);

        if (encapsulating_lanes->count() != val.count()) return false;

        size_t i, j;

        for (i = 0; i < encapsulating_lanes->count(); ++i)
        {
            for (j = 0; j < val.count(); ++i)
            {
                if ((*encapsulating_lanes)[i]->get_id() == val[j])
                {
                    break;
                }
            }

            if (j == val.count()) return false;
        }
    }
    return true;
}

}
}
}
