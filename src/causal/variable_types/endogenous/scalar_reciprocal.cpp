
#include <ori/simcars/causal/variable_types/endogenous/scalar_reciprocal.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE ScalarReciprocalVariable::get_value() const
{
    return 1.0 / get_parent()->get_value();
}

}
}
}
