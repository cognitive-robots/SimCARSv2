#pragma once

#include <ori/simcars/causal/exogenous_variable_interface.hpp>
#include <ori/simcars/agents/ped_reward_parameters.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class PedRewardParametersFixedVariable : public simcars::causal::IExogenousVariable<PedRewardParameters>
{
    PedRewardParameters value;

public:
    PedRewardParametersFixedVariable(PedRewardParameters value);

    bool get_value(PedRewardParameters &val) const override;

    bool set_value(PedRewardParameters const &val) override;
};

}
}
}
}
