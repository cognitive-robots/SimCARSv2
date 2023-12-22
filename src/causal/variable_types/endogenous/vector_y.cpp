
#include <ori/simcars/causal/variable_types/endogenous/vector_y.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorYVariable::get_value(FP_DATA_TYPE &val) const
{
    geometry::Vec vec;
    if (get_parent()->get_value(vec))
    {
        val = vec.y();
        return true;
    }
    else
    {
        return false;
    }
}

bool VectorYVariable::set_value(FP_DATA_TYPE const &val)
{
    geometry::Vec vec;
    if (get_parent()->get_value(vec))
    {
        vec.y() = val;
        return get_parent()->set_value(vec);
    }
    else
    {
        return false;
    }
}

}
}
}
