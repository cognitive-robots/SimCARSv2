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

    IVariable<temporal::Time> *parent;

public:
    TimeSocketVariable(temporal::Time default_value = temporal::Time(temporal::Duration(0.0)),
                       IVariable<temporal::Time> *parent = nullptr);

    bool get_value(temporal::Time &val) const override;

    IVariable<temporal::Time> const* get_parent() const;

    bool set_value(temporal::Time const &val) override;

    void set_parent(IVariable<temporal::Time> *parent);
};

}
}
}
