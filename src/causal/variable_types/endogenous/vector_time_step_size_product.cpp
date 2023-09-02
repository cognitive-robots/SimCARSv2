
#include <ori/simcars/causal/variable_types/endogenous/vector_time_step_size_product.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

geometry::Vec VectorTimeStepSizeProductVariable::get_value() const
{
    return get_parent()->get_value() * VariableContext::get_time_step_size().count();
}

}
}
}
