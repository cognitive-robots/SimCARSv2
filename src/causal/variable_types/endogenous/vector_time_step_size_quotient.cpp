
#include <ori/simcars/causal/variable_types/endogenous/vector_time_step_size_quotient.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

geometry::Vec VectorTimeStepSizeQuotientVariable::get_value() const
{
    return get_parent()->get_value() / VariableContext::get_time_step_size().count();
}

}
}
}
