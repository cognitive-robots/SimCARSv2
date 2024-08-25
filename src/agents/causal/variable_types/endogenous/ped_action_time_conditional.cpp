
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_action_time_conditional.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

PedActionTimeConditionalVariable::PedActionTimeConditionalVariable(
        IEndogenousVariable<PedAction> *endogenous_parent, IVariable<PedAction> *other_parent,
        temporal::Time time) : ABinaryEndogenousVariable(endogenous_parent, other_parent),
    time(time) {}

bool PedActionTimeConditionalVariable::get_value(PedAction &val) const
{
    if (time > simcars::causal::VariableContext::get_current_time())
    {
        return get_endogenous_parent()->get_value(val);
    }
    else
    {
        return get_other_parent()->get_value(val);
    }
}

bool PedActionTimeConditionalVariable::set_value(PedAction const &val)
{
    if (time > simcars::causal::VariableContext::get_current_time())
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
}
