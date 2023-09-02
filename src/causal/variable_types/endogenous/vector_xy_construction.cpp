
#include <ori/simcars/causal/variable_types/endogenous/vector_xy_construction.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

geometry::Vec VectorXYConstructionVariable::get_value() const
{
    return geometry::Vec(get_endogenous_parent()->get_value(), get_other_parent()->get_value());
}

}
}
}
