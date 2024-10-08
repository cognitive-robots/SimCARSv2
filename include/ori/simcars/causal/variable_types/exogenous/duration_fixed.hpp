#pragma once

#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/causal/exogenous_variable_interface.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class DurationFixedVariable : public IExogenousVariable<temporal::Duration>
{
    temporal::Duration value;

public:
    DurationFixedVariable(temporal::Duration value);

    bool get_value(temporal::Duration &val) const override;

    bool set_value(temporal::Duration const &val) override;
};

}
}
}
