
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_task_proxy.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool PedTaskProxyVariable::get_value(PedTask &val) const
{
    return get_parent()->get_value(val);
}

bool PedTaskProxyVariable::set_value(PedTask const &val)
{
    return get_parent()->set_value(val);
}

}
}
}
}
