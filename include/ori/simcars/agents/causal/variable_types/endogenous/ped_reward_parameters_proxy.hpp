#pragma once

#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/ped_reward_parameters.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class PedRewardParametersProxyVariable :
        public simcars::causal::AUnaryEndogenousVariable<PedRewardParameters, PedRewardParameters>
{
public:
    using AUnaryEndogenousVariable<PedRewardParameters, PedRewardParameters>::AUnaryEndogenousVariable;

    bool get_value(PedRewardParameters &val) const override;

    bool set_value(PedRewardParameters const &val) override;
};

}
}
}
}
