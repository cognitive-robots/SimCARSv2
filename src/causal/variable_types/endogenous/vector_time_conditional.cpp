
#include <ori/simcars/causal/variable_types/endogenous/vector_time_conditional.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

VectorTimeConditionalVariable::VectorTimeConditionalVariable(
        IEndogenousVariable<geometry::Vec> *endogenous_parent,
        IVariable<geometry::Vec> *other_parent, temporal::Time time) :
    ABinaryEndogenousVariable(endogenous_parent, other_parent), time(time) {}

bool VectorTimeConditionalVariable::get_value(geometry::Vec &val) const
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

bool VectorTimeConditionalVariable::set_value(geometry::Vec const &val)
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
