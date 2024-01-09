
#include <ori/simcars/causal/variable_types/endogenous/o_rect_construction.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ORectConstructionVariable::get_value(geometry::ORect &val) const
{
    geometry::Vec pos;
    FP_DATA_TYPE width, height, rot;
    if (get_endogenous_parent_1()->get_value(pos) && get_endogenous_parent_2()->get_value(rot) &&
            get_endogenous_parent_3()->get_value(width) && get_other_parent()->get_value(height))
    {
        val = geometry::ORect(pos, width, height, rot);
        return true;
    }
    else
    {
        return false;
    }
}

bool ORectConstructionVariable::set_value(geometry::ORect const &val)
{
    // WARNING: Cannot undo setting of earlier variables in the chain
    return get_endogenous_parent_1()->set_value(val.get_origin()) &&
            get_endogenous_parent_2()->set_value(val.get_orientation()) &&
            get_endogenous_parent_3()->set_value(val.get_width()) &&
            get_other_parent()->set_value(val.get_height());
}

}
}
}
