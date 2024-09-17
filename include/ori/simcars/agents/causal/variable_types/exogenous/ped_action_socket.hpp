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

class PedActionSocketVariable : public simcars::causal::IExogenousVariable<PedAction>
{
    PedAction default_value;

    IVariable<PedAction> *parent;

public:
    PedActionSocketVariable(PedAction default_value = PedAction(),
                               IVariable<PedAction> *parent = nullptr);

    bool get_value(PedAction &val) const override;

    IVariable<PedAction> const* get_parent() const;

    bool set_value(PedAction const &val) override;

    void set_parent(IVariable<PedAction> *parent);
};

}
}
}
}
