
#include <ori/simcars/agents/causal/variable_types/exogenous/ped_task_socket.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

PedTaskSocketVariable::PedTaskSocketVariable(PedTask default_value, IVariable<PedTask> *parent) :
    default_value(default_value), parent(parent) {}

bool PedTaskSocketVariable::get_value(PedTask &val) const
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

simcars::causal::IVariable<PedTask> const* PedTaskSocketVariable::get_parent() const
{
    return parent;
}

bool PedTaskSocketVariable::set_value(PedTask const &val)
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

void PedTaskSocketVariable::set_parent(IVariable<PedTask> *parent)
{
    this->parent = parent;
}

}
}
}
}
