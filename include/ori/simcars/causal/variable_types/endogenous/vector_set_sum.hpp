#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/causal/set_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorSetSumVariable : public ASetEndogenousVariable<geometry::Vec, geometry::Vec>
{
public:
    using ASetEndogenousVariable<geometry::Vec, geometry::Vec>::ASetEndogenousVariable;

    geometry::Vec get_value() const override;
};

}
}
}
