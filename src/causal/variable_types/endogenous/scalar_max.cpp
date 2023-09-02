
#include <ori/simcars/causal/variable_types/endogenous/scalar_max.hpp>

#include <algorithm>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE ScalarMaxVariable::get_value() const
{
    return std::max(get_endogenous_parent()->get_value(), get_other_parent()->get_value());
}

}
}
}
