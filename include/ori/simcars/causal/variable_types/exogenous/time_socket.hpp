#pragma once

#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/causal/exogenous_variable_interface.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class TimeSocketVariable : public IExogenousVariable<temporal::Time>
{
    temporal::Time default_value;

    IVariable<temporal::Time> const *parent;

public:
    TimeSocketVariable(temporal::Time default_value = temporal::Time(temporal::Duration(0.0)),
                       IVariable<temporal::Time> const *parent = nullptr);

    temporal::Time get_value() const override;

    IVariable<temporal::Time> const* get_parent() const;

    void set_parent(IVariable<temporal::Time> const *parent);
};

}
}
}
