
#include <ori/simcars/causal/variable_types/endogenous/scalar_conditional.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE ScalarConditionalVariable::get_value() const
{
    return get_other_parent()->get_value() ? get_endogenous_parent_1()->get_value() :
                                             get_endogenous_parent_2()->get_value();
}

}
}
}
