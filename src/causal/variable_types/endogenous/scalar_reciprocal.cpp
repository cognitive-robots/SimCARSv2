
#include <ori/simcars/causal/variable_types/endogenous/scalar_reciprocal.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ScalarReciprocalVariable::get_value(FP_DATA_TYPE &val) const
{
    if (get_parent()->get_value(val))
    {
        val = 1.0 / val;
        return true;
    }
    else
    {
        return false;
    }
}

bool ScalarReciprocalVariable::set_value(FP_DATA_TYPE const &val)
{
    return get_parent()->set_value(1.0 / val);
}
}
}
}
