
#include <ori/simcars/causal/variable_types/endogenous/scalar_absolute.hpp>

#include <cmath>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ScalarAbsoluteVariable::get_value(FP_DATA_TYPE &val) const
{
    if (get_parent()->get_value(val))
    {
        val = std::abs(val);
        return true;
    }
    else
    {
        return false;
    }
}

bool ScalarAbsoluteVariable::set_value(FP_DATA_TYPE const &val)
{
    FP_DATA_TYPE val_1;
    if (get_parent()->get_value(val_1))
    {
        return val == val_1;
    }
    else
    {
        return true;
    }
}

}
}
}
