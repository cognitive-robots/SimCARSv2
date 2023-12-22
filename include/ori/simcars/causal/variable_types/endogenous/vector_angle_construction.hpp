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
    VectorAngleConstructionVariable(IVariable<FP_DATA_TYPE> *parent);

    bool get_value(geometry::Vec &val) const override;

    bool set_value(geometry::Vec const &val) override;
};

}
}
}
