
#include <ori/simcars/causal/variable_types/endogenous/ids_previous_time_step.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool IdsPreviousTimeStepVariable::get_value(structures::stl::STLStackArray<uint64_t> &val) const
{
    VariableContext::decrement_time_step();
    bool res = get_parent()->get_value(val);
    VariableContext::increment_time_step();
    return res;
}

bool IdsPreviousTimeStepVariable::set_value(structures::stl::STLStackArray<uint64_t> const &val)
{
    VariableContext::decrement_time_step();
    bool res = get_parent()->set_value(val);
    VariableContext::increment_time_step();
    return res;
}

}
}
}
