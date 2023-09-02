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
    geometry::Vec const value;

public:
    VectorFixedVariable(geometry::Vec value);

    geometry::Vec get_value() const override;
};

}
}
}
