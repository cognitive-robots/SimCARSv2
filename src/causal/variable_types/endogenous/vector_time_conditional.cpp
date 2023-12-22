
#include <ori/simcars/causal/variable_types/endogenous/vector_time_conditional.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorTimeConditionalVariable::get_value(geometry::Vec &val) const
{
    temporal::Time time;
    if (get_other_parent()->get_value(time))
    {
        if (time > VariableContext::get_current_time())
        {
            return get_endogenous_parent_1()->get_value(val);
        }
        else
        {
            return get_endogenous_parent_2()->get_value(val);
        }
    }
    else
    {
        return false;
    }
}

bool VectorTimeConditionalVariable::set_value(geometry::Vec const &val)
{
    // TODO: Allow set to change time variable
    temporal::Time time;
    if (get_other_parent()->get_value(time))
    {
        if (time > VariableContext::get_current_time())
        {
            return get_endogenous_parent_1()->set_value(val);
        }
        else
        {
            return get_endogenous_parent_2()->set_value(val);
        }
    }
    else
    {
        return false;
    }
}

}
}
}
