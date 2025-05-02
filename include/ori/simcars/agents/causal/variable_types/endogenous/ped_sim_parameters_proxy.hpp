#pragma once

#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/ped_sim_parameters.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class PedSimParametersProxyVariable :
        public simcars::causal::AUnaryEndogenousVariable<PedSimParameters, PedSimParameters>
{
public:
    using AUnaryEndogenousVariable<PedSimParameters, PedSimParameters>::AUnaryEndogenousVariable;

    bool get_value(PedSimParameters &val) const override;

    bool set_value(PedSimParameters const &val) override;
};

}
}
}
}
