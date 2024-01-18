
#include <ori/simcars/causal/variable_types/endogenous/scalar_time_step_size_product.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ScalarTimeStepSizeProductVariable::get_value(FP_DATA_TYPE &val) const
{
    if (get_parent()->get_value(val))
    {
        val = val * std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(
                    VariableContext::get_time_step_size()).count();
        return true;
    }
    else
    {
        return false;
    }
}

bool ScalarTimeStepSizeProductVariable::set_value(FP_DATA_TYPE const &val)
{
    return get_parent()->set_value(val /
                                   std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(
                                       VariableContext::get_time_step_size()).count());
}

}
}
}
