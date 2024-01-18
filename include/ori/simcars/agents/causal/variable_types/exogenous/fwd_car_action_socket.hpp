#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/exogenous_variable_interface.hpp>
#include <ori/simcars/agents/fwd_car_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class FWDCarActionSocketVariable : public simcars::causal::IExogenousVariable<FWDCarAction>
{
    FWDCarAction default_value;

    IVariable<FWDCarAction> *parent;

public:
    FWDCarActionSocketVariable(FWDCarAction default_value = FWDCarAction(),
                               IVariable<FWDCarAction> *parent = nullptr);

    bool get_value(FWDCarAction &val) const override;

    IVariable<FWDCarAction> const* get_parent() const;

    bool set_value(FWDCarAction const &val) override;

    void set_parent(IVariable<FWDCarAction> *parent);
};

}
}
}
}
