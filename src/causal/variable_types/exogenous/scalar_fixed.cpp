
#include <ori/simcars/causal/variable_types/exogenous/scalar_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

ScalarFixedVariable::ScalarFixedVariable(FP_DATA_TYPE value) : value(value) {}

FP_DATA_TYPE ScalarFixedVariable::get_value() const
{
    return value;
}

}
}
}
