#pragma once

#include <ori/simcars/causal/exogenous_variable_interface.hpp>
#include <ori/simcars/agents/ped_sim_parameters.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class PedSimParametersFixedVariable : public simcars::causal::IExogenousVariable<PedSimParameters>
{
    PedSimParameters value;

public:
    PedSimParametersFixedVariable(PedSimParameters value);

    bool get_value(PedSimParameters &val) const override;

    bool set_value(PedSimParameters const &val) override;
};

}
}
}
}
