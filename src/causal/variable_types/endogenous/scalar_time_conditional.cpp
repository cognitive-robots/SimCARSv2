
#include <ori/simcars/causal/variable_types/endogenous/scalar_time_conditional.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

ScalarTimeConditionalVariable::ScalarTimeConditionalVariable(
        IEndogenousVariable<FP_DATA_TYPE> *endogenous_parent, IVariable<FP_DATA_TYPE> *other_parent,
        temporal::Time time) : ABinaryEndogenousVariable(endogenous_parent, other_parent),
    time(time) {}

bool ScalarTimeConditionalVariable::get_value(FP_DATA_TYPE &val) const
{
    if (time >= VariableContext::get_current_time())
    {
        return get_endogenous_parent()->get_value(val);
    }
    else
    {
        return get_other_parent()->get_value(val);
    }
}

bool ScalarTimeConditionalVariable::set_value(FP_DATA_TYPE const &val)
{
    if (time >= VariableContext::get_current_time())
    {
        return get_endogenous_parent()->set_value(val);
    }
    else
    {
        return get_other_parent()->set_value(val);
    }
}

}
}
}
