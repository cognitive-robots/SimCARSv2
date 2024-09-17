
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_time_conditional.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

FWDCarActionTimeConditionalVariable::FWDCarActionTimeConditionalVariable(
        IEndogenousVariable<FWDCarAction> *endogenous_parent, IVariable<FWDCarAction> *other_parent,
        temporal::Time time) : ABinaryEndogenousVariable(endogenous_parent, other_parent),
    time(time) {}

bool FWDCarActionTimeConditionalVariable::get_value(FWDCarAction &val) const
{
    if (time >= simcars::causal::VariableContext::get_current_time())
    {
        return get_endogenous_parent()->get_value(val);
    }
    else
    {
        return get_other_parent()->get_value(val);
    }
}

bool FWDCarActionTimeConditionalVariable::set_value(FWDCarAction const &val)
{
    if (time >= simcars::causal::VariableContext::get_current_time())
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
