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
    temporal::Duration const value;

public:
    DurationFixedVariable(temporal::Duration value);

    temporal::Duration get_value() const override;
};

}
}
}
