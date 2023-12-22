#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/causal/exogenous_variable_interface.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorFixedVariable : public IExogenousVariable<geometry::Vec>
{
    geometry::Vec value;

public:
    VectorFixedVariable(geometry::Vec value);

    bool get_value(geometry::Vec &val) const override;

    bool set_value(geometry::Vec const &val) override;
};

}
}
}
