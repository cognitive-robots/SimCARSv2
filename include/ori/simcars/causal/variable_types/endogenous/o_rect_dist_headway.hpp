#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/geometry/o_rect.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ORectDistHeadwayVariable :
        public ABinaryEndogenousVariable<FP_DATA_TYPE, geometry::ORect, geometry::ORect>
{
    geometry::TrigBuff const* trig_buff;

public:
    ORectDistHeadwayVariable(IEndogenousVariable<geometry::ORect> *endogenous_parent,
                             IVariable<geometry::ORect> *other_parent);

    bool get_value(FP_DATA_TYPE &val) const override;

    bool set_value(FP_DATA_TYPE const &val) override;
};

}
}
}
