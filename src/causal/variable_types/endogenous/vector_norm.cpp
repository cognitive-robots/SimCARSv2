
#include <ori/simcars/causal/variable_types/endogenous/vector_norm.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorNormVariable::get_value(FP_DATA_TYPE &val) const
{
    geometry::Vec vec;
    if (get_parent()->get_value(vec))
    {
        val = vec.norm();
        return true;
    }
    else
    {
        return false;
    }
}

bool VectorNormVariable::set_value(FP_DATA_TYPE const &val)
{
    geometry::Vec vec;
    if (get_parent()->get_value(vec))
    {
        return val == vec.norm();
    }
    else
    {
        return true;
    }
}

}
}
}
