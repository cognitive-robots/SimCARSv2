#pragma once

#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/fwd_car_reward_parameters.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class FWDCarRewardParametersProxyVariable :
        public simcars::causal::AUnaryEndogenousVariable<FWDCarRewardParameters, FWDCarRewardParameters>
{
public:
    using AUnaryEndogenousVariable<FWDCarRewardParameters, FWDCarRewardParameters>::AUnaryEndogenousVariable;

    bool get_value(FWDCarRewardParameters &val) const override;

    bool set_value(FWDCarRewardParameters const &val) override;
};

}
}
}
}
