
#include <ori/simcars/causal/variable_types/exogenous/scalar_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

ScalarFixedVariable::ScalarFixedVariable(FP_DATA_TYPE value) : value(value) {}

bool ScalarFixedVariable::get_value(FP_DATA_TYPE &val) const
{
    val = value;
    return true;
}

bool ScalarFixedVariable::set_value(FP_DATA_TYPE const &val)
{
    value = val;
    return true;
}

}
}
}
