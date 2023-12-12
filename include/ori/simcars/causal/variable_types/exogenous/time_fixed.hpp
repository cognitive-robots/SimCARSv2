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
    temporal::Time const value;

public:
    TimeFixedVariable(temporal::Time value);

    temporal::Time get_value() const override;
};

}
}
}
