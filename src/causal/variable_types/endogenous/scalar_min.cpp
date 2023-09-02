
#include <ori/simcars/causal/variable_types/endogenous/scalar_min.hpp>

#include <algorithm>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE ScalarMinVariable::get_value() const
{
    return std::min(get_endogenous_parent()->get_value(), get_other_parent()->get_value());
}

}
}
}
