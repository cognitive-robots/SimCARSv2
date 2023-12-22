
#include <ori/simcars/causal/variable_types/endogenous/scalar_negation.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ScalarNegationVariable::get_value(FP_DATA_TYPE &val) const
{
    if (get_parent()->get_value(val))
    {
        val = -val;
        return true;
    }
    else
    {
        return false;
    }
}

bool ScalarNegationVariable::set_value(FP_DATA_TYPE const &val)
{
    return get_parent()->set_value(-val);
}

}
}
}
