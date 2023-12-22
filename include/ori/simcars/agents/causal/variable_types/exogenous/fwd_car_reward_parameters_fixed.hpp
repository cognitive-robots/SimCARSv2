#pragma once

#include <ori/simcars/causal/exogenous_variable_interface.hpp>
#include <ori/simcars/agents/fwd_car_reward_parameters.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class FWDCarRewardParametersFixedVariable : public simcars::causal::IExogenousVariable<FWDCarRewardParameters>
{
    FWDCarRewardParameters const value;

public:
    FWDCarRewardParametersFixedVariable(FWDCarRewardParameters value);

    bool get_value(FWDCarRewardParameters &val) const override;
};

}
}
}
}
