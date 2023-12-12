#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/causal/ternary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorTimeConditionalVariable :
        public ATernaryEndogenousVariable<geometry::Vec, geometry::Vec, geometry::Vec, temporal::Time>
{
public:
    using ATernaryEndogenousVariable<geometry::Vec, geometry::Vec, geometry::Vec,
    temporal::Time>::ATernaryEndogenousVariable;

    geometry::Vec get_value() const override;
};

}
}
}
