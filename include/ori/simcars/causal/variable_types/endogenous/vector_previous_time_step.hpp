#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorPreviousTimeStepVariable : public AUnaryEndogenousVariable<geometry::Vec, geometry::Vec>
{
public:
    using AUnaryEndogenousVariable<geometry::Vec, geometry::Vec>::AUnaryEndogenousVariable;

    geometry::Vec get_value() const override;
};

}
}
}
