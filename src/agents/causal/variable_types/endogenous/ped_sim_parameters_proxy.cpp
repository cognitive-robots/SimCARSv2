
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_sim_parameters_proxy.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool PedSimParametersProxyVariable::get_value(PedSimParameters &val) const
{
    return get_parent()->get_value(val);
}

bool PedSimParametersProxyVariable::set_value(PedSimParameters const &val)
{
    return get_parent()->set_value(val);
}

}
}
}
}
