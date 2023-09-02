
#include <ori/simcars/causal/variable_types/endogenous/matrix_angle_construction.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

MatrixAngleConstructionVariable::MatrixAngleConstructionVariable(
        IVariable<FP_DATA_TYPE> const *parent) : AUnaryEndogenousVariable(parent),
    trig_buff(geometry::TrigBuff::get_instance()) {}

geometry::RotMat MatrixAngleConstructionVariable::get_value() const
{
    return trig_buff->get_rot_mat(get_parent()->get_value());
}

}
}
}
