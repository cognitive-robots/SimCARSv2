
#include <ori/simcars/causal/variable_types/endogenous/scalar_negation.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE ScalarNegationVariable::get_value() const
{
    return -get_parent()->get_value();
}

}
}
}
