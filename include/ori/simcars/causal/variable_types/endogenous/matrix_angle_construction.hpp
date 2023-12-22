#pragma once

#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class MatrixAngleConstructionVariable : public AUnaryEndogenousVariable<geometry::RotMat, FP_DATA_TYPE>
{
    geometry::TrigBuff const* trig_buff;

public:
    MatrixAngleConstructionVariable(IVariable<FP_DATA_TYPE> *parent);

    bool get_value(geometry::RotMat &val) const override;

    bool set_value(geometry::RotMat const &val) override;
};

}
}
}
