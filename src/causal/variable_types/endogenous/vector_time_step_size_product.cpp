
#include <ori/simcars/causal/variable_types/endogenous/vector_time_step_size_product.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorTimeStepSizeProductVariable::get_value(geometry::Vec &val) const
{
    if (get_parent()->get_value(val))
    {
        val = val * std::chrono::duration_cast<std::chrono::seconds>(
                    VariableContext::get_time_step_size()).count();
        return true;
    }
    else
    {
        return false;
    }
}

bool VectorTimeStepSizeProductVariable::set_value(geometry::Vec const &val)
{
    return get_parent()->set_value(val / std::chrono::duration_cast<std::chrono::seconds>(
                                       VariableContext::get_time_step_size()).count());
}

}
}
}
