#pragma once

#include <ori/simcars/geometry/o_rect.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ORectContactVariable :
        public ABinaryEndogenousVariable<geometry::VecPair, geometry::ORect, geometry::ORect>
{
public:
    using ABinaryEndogenousVariable<geometry::VecPair, geometry::ORect, geometry::ORect>::ABinaryEndogenousVariable;

    bool get_value(geometry::VecPair &val) const override;

    bool set_value(geometry::VecPair const &val) override;
};

}
}
}
