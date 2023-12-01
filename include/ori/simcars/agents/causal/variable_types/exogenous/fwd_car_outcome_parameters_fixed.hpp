#pragma once

#include <ori/simcars/causal/exogenous_variable_interface.hpp>
#include <ori/simcars/agents/fwd_car_sim_parameters.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class FWDCarSimParametersFixedVariable : public simcars::causal::IExogenousVariable<FWDCarSimParameters>
{
    FWDCarSimParameters const value;

public:
    FWDCarSimParametersFixedVariable(FWDCarSimParameters value);

    FWDCarSimParameters get_value() const override;
};

}
}
}
}
