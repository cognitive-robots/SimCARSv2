
#include <ori/simcars/causal/variable_types/endogenous/scalar_previous_time_step.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ScalarPreviousTimeStepVariable::get_value(FP_DATA_TYPE &val) const
{
    VariableContext::decrement_time_step();
    temporal::Time current_time = VariableContext::get_current_time();
    bool res = get_parent()->get_value(val);
    VariableContext::increment_time_step();
    return res;
}

bool ScalarPreviousTimeStepVariable::set_value(FP_DATA_TYPE const &val)
{
    VariableContext::decrement_time_step();
    bool res = get_parent()->set_value(val);
    VariableContext::increment_time_step();
    return res;
}

}
}
}
