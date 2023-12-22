
#include <ori/simcars/causal/variable_types/endogenous/vector_angle_construction.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

VectorAngleConstructionVariable::VectorAngleConstructionVariable(
        IVariable<FP_DATA_TYPE> *parent) : AUnaryEndogenousVariable(parent),
    trig_buff(geometry::TrigBuff::get_instance()) {}

bool VectorAngleConstructionVariable::get_value(geometry::Vec &val) const
{
    FP_DATA_TYPE angle;
    if (get_parent()->get_value(angle))
    {
        FP_DATA_TYPE sin_o = trig_buff->get_sin(angle);
        FP_DATA_TYPE cos_o = trig_buff->get_cos(angle);
        val = geometry::Vec(cos_o, sin_o);
        return true;
    }
    else
    {
        return false;
    }
}

bool VectorAngleConstructionVariable::set_value(geometry::Vec const &val)
{
    return get_parent()->set_value(std::atan2(val.y(), val.x()));
}

}
}
}
