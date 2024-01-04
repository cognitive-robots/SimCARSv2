
#include <ori/simcars/causal/variable_types/endogenous/vector_x.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorXVariable::get_value(FP_DATA_TYPE &val) const
{
    geometry::Vec vec;
    if (get_parent()->get_value(vec))
    {
        val = vec.x();
        return true;
    }
    else
    {
        return false;
    }
}

bool VectorXVariable::set_value(FP_DATA_TYPE const &val)
{
    geometry::Vec vec;
    if (get_parent()->get_value(vec))
    {
        vec.x() = val;
        return get_parent()->set_value(vec);
    }
    else
    {
        return true;
    }
}

}
}
}
