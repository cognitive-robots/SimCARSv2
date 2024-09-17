
#include <ori/simcars/agents/causal/variable_types/exogenous/ped_action_socket.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

PedActionSocketVariable::PedActionSocketVariable(PedAction default_value,
                                                 IVariable<PedAction> *parent) :
    default_value(default_value), parent(parent) {}

bool PedActionSocketVariable::get_value(PedAction &val) const
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

simcars::causal::IVariable<PedAction> const* PedActionSocketVariable::get_parent() const
{
    return parent;
}

bool PedActionSocketVariable::set_value(PedAction const &val)
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

void PedActionSocketVariable::set_parent(IVariable<PedAction> *parent)
{
    this->parent = parent;
}

}
}
}
}
