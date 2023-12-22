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

    IVariable<geometry::Vec> *parent;

public:
    VectorSocketVariable(geometry::Vec default_value = geometry::Vec::Zero(),
                         IVariable<geometry::Vec> *parent = nullptr);

    bool get_value(geometry::Vec &val) const override;

    IVariable<geometry::Vec> const* get_parent() const;

    bool set_value(geometry::Vec const &val) override;

    void set_parent(IVariable<geometry::Vec> *parent);
};

}
}
}
