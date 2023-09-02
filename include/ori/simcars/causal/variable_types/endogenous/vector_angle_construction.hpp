#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorAngleConstructionVariable : public AUnaryEndogenousVariable<geometry::Vec, FP_DATA_TYPE>
{
    geometry::TrigBuff const* trig_buff;

public:
    VectorAngleConstructionVariable(IVariable<FP_DATA_TYPE> const *parent);

    geometry::Vec get_value() const override;
};

}
}
}
