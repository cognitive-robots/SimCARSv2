
#include <ori/simcars/causal/variable_types/endogenous/vector_scalar_product.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorScalarProductVariable::get_value(geometry::Vec &val) const
{
    FP_DATA_TYPE other_val;
    if (get_other_parent()->get_value(other_val) && get_endogenous_parent()->get_value(val))
    {
        val = val * other_val;
        return true;
    }
    else
    {
        return false;
    }
}

bool VectorScalarProductVariable::set_value(geometry::Vec const &val)
{
    geometry::Vec vec;
    FP_DATA_TYPE scalar;
    if (get_endogenous_parent()->get_value(vec))
    {
        if (get_other_parent()->get_value(scalar))
        {
            return val == vec * scalar;
        }
        else
        {
            return get_other_parent()->set_value(val.norm() / vec.norm());
        }
    }
    else
    {
        if (get_other_parent()->get_value(scalar))
        {
            return get_endogenous_parent()->set_value(val / scalar);
        }
        else
        {
            return true;
        }
    }
}

}
}
}
