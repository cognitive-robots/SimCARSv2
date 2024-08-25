
#include <ori/simcars/agents/causal/variable_types/exogenous/ped_action_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

PedActionFixedVariable::PedActionFixedVariable(PedAction value) : value(value) {}

bool PedActionFixedVariable::get_value(PedAction &val) const
{
    val = value;
    return true;
}

bool PedActionFixedVariable::set_value(PedAction const &val)
{
    value = val;
    return true;
}

}
}
}
}
