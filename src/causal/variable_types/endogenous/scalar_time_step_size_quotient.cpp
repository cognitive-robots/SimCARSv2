
#include <ori/simcars/causal/variable_types/endogenous/scalar_time_step_size_quotient.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ScalarTimeStepSizeQuotientVariable::get_value(FP_DATA_TYPE &val) const
{
    if (get_parent()->get_value(val))
    {
        val = val / std::chrono::duration_cast<std::chrono::seconds>(
                    VariableContext::get_time_step_size()).count();
        return true;
    }
    else
    {
        return false;
    }
}

bool ScalarTimeStepSizeQuotientVariable::set_value(FP_DATA_TYPE const &val)
{
    return get_parent()->set_value(val * std::chrono::duration_cast<std::chrono::seconds>(
                                       VariableContext::get_time_step_size()).count());
}

}
}
}
