
#include <ori/simcars/causal/variable_types/endogenous/matrix_angle_construction.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

MatrixAngleConstructionVariable::MatrixAngleConstructionVariable(IVariable<FP_DATA_TYPE> *parent) :
    AUnaryEndogenousVariable(parent), trig_buff(geometry::TrigBuff::get_instance()) {}

bool MatrixAngleConstructionVariable::get_value(geometry::RotMat &val) const
{
    FP_DATA_TYPE angle;
    if (get_parent()->get_value(angle))
    {
        val = trig_buff->get_rot_mat(angle);
        return true;
    }
    else
    {
        return false;
    }
}

bool MatrixAngleConstructionVariable::set_value(geometry::RotMat const &val)
{
    FP_DATA_TYPE angle = std::atan2(val(1,0), val(0,0));
    return get_parent()->set_value(angle);
}

}
}
}
