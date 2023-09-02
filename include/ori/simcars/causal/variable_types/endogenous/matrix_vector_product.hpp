#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class MatrixVectorProductVariable :
        public ABinaryEndogenousVariable<geometry::Vec, geometry::RotMat, geometry::Vec>
{
public:
    using ABinaryEndogenousVariable<geometry::Vec, geometry::RotMat, geometry::Vec>::ABinaryEndogenousVariable;

    geometry::Vec get_value() const override;
};

}
}
}
