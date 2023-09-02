
#include <ori/simcars/causal/variable_types/endogenous/scalar_time_step_size_product.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE ScalarTimeStepSizeProductVariable::get_value() const
{
    return get_parent()->get_value() * std::chrono::duration_cast<std::chrono::seconds>(
                VariableContext::get_time_step_size()).count();
}

}
}
}
