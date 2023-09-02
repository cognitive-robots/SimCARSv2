#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/causal/ternary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorConditionalVariable :
        public ATernaryEndogenousVariable<geometry::Vec, geometry::Vec, geometry::Vec, bool>
{
public:
    using ATernaryEndogenousVariable<geometry::Vec, geometry::Vec, geometry::Vec, bool>::ATernaryEndogenousVariable;

    geometry::Vec get_value() const override;
};

}
}
}
