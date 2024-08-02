
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_reward_parameters_proxy.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool PedRewardParametersProxyVariable::get_value(PedRewardParameters &val) const
{
    return get_parent()->get_value(val);
}

bool PedRewardParametersProxyVariable::set_value(PedRewardParameters const &val)
{
    return get_parent()->set_value(val);
}

}
}
}
}
