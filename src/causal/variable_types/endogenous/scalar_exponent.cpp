
#include <ori/simcars/causal/variable_types/endogenous/scalar_exponent.hpp>

#include <cmath>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ScalarExponentVariable::get_value(FP_DATA_TYPE &val) const
{
    if (get_parent()->get_value(val))
    {
        val = std::exp(val);
        return true;
    }
    else
    {
        return false;
    }
}

bool ScalarExponentVariable::set_value(FP_DATA_TYPE const &val)
{
    return get_parent()->set_value(std::log(val));
}

}
}
}
