#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/causal/exogenous_variable_interface.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorSocketVariable : public IExogenousVariable<geometry::Vec>
{
    geometry::Vec default_value;

    IVariable<geometry::Vec> const *parent;

public:
    VectorSocketVariable(geometry::Vec default_value = geometry::Vec::Zero(),
                         IVariable<geometry::Vec> const *parent = nullptr);

    geometry::Vec get_value() const override;

    IVariable<geometry::Vec> const* get_parent() const;

    void set_parent(IVariable<geometry::Vec> const *parent);
};

}
}
}
