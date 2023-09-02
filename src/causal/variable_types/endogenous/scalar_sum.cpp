
#include <ori/simcars/causal/variable_types/endogenous/scalar_sum.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE ScalarSumVariable::get_value() const
{
    return get_endogenous_parent()->get_value() + get_other_parent()->get_value();
}

}
}
}
