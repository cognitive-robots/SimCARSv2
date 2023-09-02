
#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

thread_local temporal::Time VariableContext::current_time = temporal::Time(temporal::Duration(0));
thread_local temporal::Duration VariableContext::time_step_size = temporal::Duration(1);

temporal::Time VariableContext::get_current_time()
{
    return VariableContext::current_time;
}

bool VariableContext::set_current_time(temporal::Time time)
{
    VariableContext::current_time = time;

    return true;
}

temporal::Duration VariableContext::get_time_step_size()
{
    return VariableContext::time_step_size;
}

bool VariableContext::set_time_step_size(temporal::Duration time_step_size)
{
    if (time_step_size > temporal::Duration(0))
    {
        VariableContext::time_step_size = time_step_size;
        return true;
    }
    else
    {
        return false;
    }
}

void VariableContext::increment_time_step()
{
    VariableContext::current_time += VariableContext::time_step_size;
}

void VariableContext::decrement_time_step()
{
    VariableContext::current_time -= VariableContext::time_step_size;
}

}
}
}
