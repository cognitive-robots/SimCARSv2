
#include <ori/simcars/causal/variable_types/endogenous/scalar_binary_mean.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ScalarBinaryMeanVariable::get_value(FP_DATA_TYPE &val) const
{
    FP_DATA_TYPE other_val;
    if (get_other_parent()->get_value(other_val) && get_endogenous_parent()->get_value(val))
    {
        val = 0.5 * (val + other_val);
        return true;
    }
    else
    {
        return false;
    }
}

bool ScalarBinaryMeanVariable::set_value(FP_DATA_TYPE const &val)
{
    FP_DATA_TYPE val_1, val_2;
    if (get_endogenous_parent()->get_value(val_1))
    {
        if (get_other_parent()->get_value(val_2))
        {
            return val == 0.5 * (val_1 + val_2);
        }
        else
        {
            return get_other_parent()->set_value((2.0 * val) - val_1);
        }
    }
    else
    {
        if (get_other_parent()->get_value(val_2))
        {
            return get_endogenous_parent()->set_value((2.0 * val) - val_2);
        }
        else
        {
            return false;
        }
    }
}

}
}
}
