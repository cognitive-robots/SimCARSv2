
#include <ori/simcars/causal/variable_types/exogenous/vector_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

VectorFixedVariable::VectorFixedVariable(geometry::Vec value) : value(value) {}

bool VectorFixedVariable::get_value(geometry::Vec &val) const
{
    val = value;
    return true;
}

bool VectorFixedVariable::set_value(geometry::Vec const &val)
{
    value = val;
    return true;
}

}
}
}
