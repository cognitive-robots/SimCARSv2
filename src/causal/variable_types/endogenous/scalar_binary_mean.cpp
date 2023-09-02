
#include <ori/simcars/causal/variable_types/endogenous/scalar_binary_mean.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE ScalarBinaryMeanVariable::get_value() const
{
    return 0.5 * (get_endogenous_parent()->get_value() + get_other_parent()->get_value());
}

}
}
}
