
#include <ori/simcars/causal/variable_types/endogenous/scalar_set_sum.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ScalarSetSumVariable::get_value(FP_DATA_TYPE &val) const
{
    structures::IArray<IEndogenousVariable<FP_DATA_TYPE>*> const *parents = get_array();
    FP_DATA_TYPE current_val, total_val = 0.0;
    bool res = false;
    for (size_t i = 0; i < count(); ++i)
    {
        if ((*parents)[i]->get_value(current_val))
        {
            total_val += current_val;
            res = true;
        }
    }

    val = total_val;
    return res;
}

bool ScalarSetSumVariable::set_value(FP_DATA_TYPE const &val)
{
    structures::IArray<IEndogenousVariable<FP_DATA_TYPE>*> const *parents = get_array();
    FP_DATA_TYPE current_val, total_val = 0.0;
    for (size_t i = 0; i < count(); ++i)
    {
        if ((*parents)[i]->get_value(current_val))
        {
            total_val += current_val;
        }
        else
        {
            // TODO: Potentially look at a better way of handling this, e.g. propogating remaining sum to undefined variable
            return true;
        }
    }

    return val == total_val;
}

}
}
}
