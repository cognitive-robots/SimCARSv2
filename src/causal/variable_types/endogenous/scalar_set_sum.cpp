
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
    for (size_t i = 0; i < count(); ++i)
    {
        if ((*parents)[i]->get_value(current_val))
        {
            total_val += current_val;
        }
        else
        {
            return false;
        }
    }

    val = total_val;
    return true;
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
            return false;
        }
    }

    return val == total_val;
}

}
}
}
