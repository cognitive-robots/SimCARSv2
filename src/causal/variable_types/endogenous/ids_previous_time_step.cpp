
#include <ori/simcars/causal/variable_types/endogenous/ids_previous_time_step.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

structures::stl::STLStackArray<uint64_t> IdsPreviousTimeStepVariable::get_value() const
{
    VariableContext::decrement_time_step();
    structures::stl::STLStackArray<uint64_t> value = get_parent()->get_value();
    VariableContext::increment_time_step();
    return value;
}

}
}
}
