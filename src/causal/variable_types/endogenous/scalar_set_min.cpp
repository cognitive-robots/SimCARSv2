
#include <ori/simcars/causal/variable_types/endogenous/scalar_set_min.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ScalarSetMinVariable::get_value(FP_DATA_TYPE &val) const
{
    structures::IArray<IEndogenousVariable<FP_DATA_TYPE>*> const *parents = get_array();
    FP_DATA_TYPE current_val, min_val = std::numeric_limits<FP_DATA_TYPE>::infinity();
    bool res = false;
    for (size_t i = 0; i < count(); ++i)
    {
        if ((*parents)[i]->get_value(current_val))
        {
            min_val = std::min(current_val, min_val);
            res = true;
        }
    }

    val = min_val;
    return res;
}

bool ScalarSetMinVariable::set_value(FP_DATA_TYPE const &val)
{
    structures::IArray<IEndogenousVariable<FP_DATA_TYPE>*> const *parents = get_array();
    FP_DATA_TYPE current_val, min_val = std::numeric_limits<FP_DATA_TYPE>::infinity();
    for (size_t i = 0; i < count(); ++i)
    {
        if ((*parents)[i]->get_value(current_val))
        {
            min_val = std::min(current_val, min_val);
        }
        else
        {
            // TODO: Potentially look at a better way of handling this, e.g. propogating min. val to undefined variable
            return true;
        }
    }

    return val == min_val;
}

}
}
}
