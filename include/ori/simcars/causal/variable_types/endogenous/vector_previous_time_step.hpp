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

    bool get_value(geometry::Vec &val) const override;

    bool set_value(geometry::Vec const &val) override;
};

}
}
}
