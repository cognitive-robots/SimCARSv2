
#include <ori/simcars/agents/causal/variable_types/exogenous/fwd_car_action_socket.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

FWDCarActionSocketVariable::FWDCarActionSocketVariable(FWDCarAction default_value,
                                                       IVariable<FWDCarAction> *parent) :
    default_value(default_value), parent(parent) {}

bool FWDCarActionSocketVariable::get_value(FWDCarAction &val) const
{
    if (parent != nullptr)
    {
        return parent->get_value(val);
    }
    else
    {
        val = default_value;
        return true;
    }
}

simcars::causal::IVariable<FWDCarAction> const* FWDCarActionSocketVariable::get_parent() const
{
    return parent;
}

bool FWDCarActionSocketVariable::set_value(FWDCarAction const &val)
{
    if (parent != nullptr)
    {
        return parent->set_value(val);
    }
    else
    {
        default_value = val;
        return true;
    }
}

void FWDCarActionSocketVariable::set_parent(IVariable<FWDCarAction> *parent)
{
    this->parent = parent;
}

}
}
}
}
