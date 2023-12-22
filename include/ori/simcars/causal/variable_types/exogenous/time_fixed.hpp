#pragma once

#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/causal/exogenous_variable_interface.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class TimeFixedVariable : public IExogenousVariable<temporal::Time>
{
    temporal::Time value;

public:
    TimeFixedVariable(temporal::Time value);

    bool get_value(temporal::Time &val) const override;

    bool set_value(temporal::Time const &val) override;
};

}
}
}
