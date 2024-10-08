
#include <ori/simcars/causal/variable_types/endogenous/vector_dot_product.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorDotProductVariable::get_value(FP_DATA_TYPE &val) const
{
    geometry::Vec val_1, val_2;
    if (get_endogenous_parent()->get_value(val_1) && get_other_parent()->get_value(val_2))
    {
        val = val_1.dot(val_2);
        return true;
    }
    else
    {
        return false;
    }
}

bool VectorDotProductVariable::set_value(FP_DATA_TYPE const &val)
{
    geometry::Vec val_1, val_2;
    if (get_endogenous_parent()->get_value(val_1) && get_other_parent()->get_value(val_2))
    {
        return val == val_1.dot(val_2);
    }
    else
    {
        return true;
    }
}

}
}
}
