
#include <ori/simcars/causal/variable_types/exogenous/vector_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

VectorFixedVariable::VectorFixedVariable(geometry::Vec value) : value(value) {}

geometry::Vec VectorFixedVariable::get_value() const
{
    return value;
}

}
}
}
