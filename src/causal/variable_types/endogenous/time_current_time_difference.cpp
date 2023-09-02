
#include <ori/simcars/causal/variable_types/endogenous/time_current_time_difference.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

temporal::Duration TimeCurrentTimeDifferenceVariable::get_value() const
{
    return get_parent()->get_value() - VariableContext::get_current_time();
}

}
}
}
