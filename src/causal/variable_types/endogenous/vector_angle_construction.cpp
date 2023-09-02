
#include <ori/simcars/causal/variable_types/endogenous/vector_angle_construction.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

VectorAngleConstructionVariable::VectorAngleConstructionVariable(
        IVariable<FP_DATA_TYPE> const *parent) : AUnaryEndogenousVariable(parent),
    trig_buff(geometry::TrigBuff::get_instance()) {}

geometry::Vec VectorAngleConstructionVariable::get_value() const
{
    FP_DATA_TYPE value = get_parent()->get_value();
    FP_DATA_TYPE sin_o = trig_buff->get_sin(value);
    FP_DATA_TYPE cos_o = trig_buff->get_cos(value);
    return geometry::Vec(cos_o, sin_o);
}

}
}
}
