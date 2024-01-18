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

class FWDCarActionFixedVariable : public simcars::causal::IExogenousVariable<FWDCarAction>
{
    FWDCarAction value;

public:
    FWDCarActionFixedVariable(FWDCarAction value);

    bool get_value(FWDCarAction &val) const override;

    bool set_value(FWDCarAction const &val) override;
};

}
}
}
}
