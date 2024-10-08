#pragma once

#include <ori/simcars/geometry/o_rect.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ORectCollisionVariable :
        public ABinaryEndogenousVariable<bool, geometry::ORect, geometry::ORect>
{
public:
    using ABinaryEndogenousVariable<bool, geometry::ORect, geometry::ORect>::ABinaryEndogenousVariable;

    bool get_value(bool &val) const override;

    bool set_value(bool const &val) override;
};

}
}
}
