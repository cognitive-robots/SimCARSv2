
#include <ori/simcars/causal/variable_types/endogenous/scalar_previous_time_step.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE ScalarPreviousTimeStepVariable::get_value() const
{
    VariableContext::decrement_time_step();
    FP_DATA_TYPE value = get_parent()->get_value();
    VariableContext::increment_time_step();
    return value;
}

}
}
}
