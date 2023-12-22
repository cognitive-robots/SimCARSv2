
#include <ori/simcars/causal/variable_types/endogenous/vector_xy_construction.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorXYConstructionVariable::get_value(geometry::Vec &val) const
{
    FP_DATA_TYPE val_1, val_2;
    if (get_endogenous_parent()->get_value(val_1) && get_other_parent()->get_value(val_2))
    {
        val = geometry::Vec(val_1, val_2);
        return true;
    }
    else
    {
        return false;
    }
}

bool VectorXYConstructionVariable::set_value(geometry::Vec const &val)
{
    return get_endogenous_parent()->set_value(val.x()) && get_other_parent()->set_value(val.y());
}

}
}
}
