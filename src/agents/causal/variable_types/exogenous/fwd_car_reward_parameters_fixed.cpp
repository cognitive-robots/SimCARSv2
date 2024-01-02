
#include <ori/simcars/agents/causal/variable_types/exogenous/fwd_car_reward_parameters_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

FWDCarRewardParametersFixedVariable::FWDCarRewardParametersFixedVariable(FWDCarRewardParameters value) :
    value(value) {}

bool FWDCarRewardParametersFixedVariable::get_value(FWDCarRewardParameters &val) const
{
    val = value;
    return true;
}

bool FWDCarRewardParametersFixedVariable::set_value(FWDCarRewardParameters const &val)
{
    value = val;
    return true;
}

}
}
}
}
