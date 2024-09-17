#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/exogenous_variable_interface.hpp>
#include <ori/simcars/agents/ped_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class PedActionFixedVariable : public simcars::causal::IExogenousVariable<PedAction>
{
    PedAction value;

public:
    PedActionFixedVariable(PedAction value);

    bool get_value(PedAction &val) const override;

    bool set_value(PedAction const &val) override;
};

}
}
}
}
