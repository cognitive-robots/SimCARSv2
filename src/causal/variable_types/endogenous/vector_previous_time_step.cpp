
#include <ori/simcars/causal/variable_types/endogenous/vector_previous_time_step.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

geometry::Vec VectorPreviousTimeStepVariable::get_value() const
{
    VariableContext::decrement_time_step();
    geometry::Vec value = get_parent()->get_value();
    VariableContext::increment_time_step();
    return value;
}

}
}
}
