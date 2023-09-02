#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorSumVariable :
        public ABinaryEndogenousVariable<geometry::Vec, geometry::Vec, geometry::Vec>
{
public:
    using ABinaryEndogenousVariable<geometry::Vec, geometry::Vec, geometry::Vec>::ABinaryEndogenousVariable;

    geometry::Vec get_value() const override;
};

}
}
}
