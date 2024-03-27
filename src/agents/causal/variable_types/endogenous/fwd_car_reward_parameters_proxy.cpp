
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_reward_parameters_proxy.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool FWDCarRewardParametersProxyVariable::get_value(FWDCarRewardParameters &val) const
{
    return get_parent()->get_value(val);
}

bool FWDCarRewardParametersProxyVariable::set_value(FWDCarRewardParameters const &val)
{
    return get_parent()->set_value(val);
}

}
}
}
}
