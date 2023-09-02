
#include <ori/simcars/causal/variable_types/endogenous/scalar_set_sum.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE ScalarSetSumVariable::get_value() const
{
    structures::IArray<IEndogenousVariable<FP_DATA_TYPE> const*> const *parents = get_array();
    FP_DATA_TYPE value;
    for (size_t i = 0; i < count(); ++i)
    {
        value += (*parents)[i]->get_value();
    }
    return value;
}

}
}
}
