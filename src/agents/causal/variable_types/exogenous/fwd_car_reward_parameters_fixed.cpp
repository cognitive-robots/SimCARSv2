
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

FWDCarRewardParameters FWDCarRewardParametersFixedVariable::get_value() const
{
    return value;
}

}
}
}
}
