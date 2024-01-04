
#include <ori/simcars/causal/variable_types/endogenous/scalar_time_conditional.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ScalarTimeConditionalVariable::get_value(FP_DATA_TYPE &val) const
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

bool ScalarTimeConditionalVariable::set_value(FP_DATA_TYPE const &val)
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
        return true;
    }
}

}
}
}
