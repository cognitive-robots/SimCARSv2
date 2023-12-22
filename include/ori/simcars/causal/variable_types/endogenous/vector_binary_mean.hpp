#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorBinaryMeanVariable :
        public ABinaryEndogenousVariable<geometry::Vec, geometry::Vec, geometry::Vec>
{
public:
    using ABinaryEndogenousVariable<geometry::Vec, geometry::Vec, geometry::Vec>::ABinaryEndogenousVariable;

    bool get_value(geometry::Vec &val) const override;

    bool set_value(geometry::Vec const &val) override;
};

}
}
}
