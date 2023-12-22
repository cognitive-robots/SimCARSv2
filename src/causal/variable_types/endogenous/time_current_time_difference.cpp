
#include <ori/simcars/causal/variable_types/endogenous/time_current_time_difference.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool TimeCurrentTimeDifferenceVariable::get_value(temporal::Duration &val) const
{
    temporal::Time time;
    if (get_parent()->get_value(time))
    {
        val = time - VariableContext::get_current_time();
        return true;
    }
    else
    {
        return false;
    }
}

bool TimeCurrentTimeDifferenceVariable::set_value(temporal::Duration const &val)
{
    return get_parent()->set_value(val + VariableContext::get_current_time());
}



}
}
}
