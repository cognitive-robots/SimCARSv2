
#include <ori/simcars/causal/variable_types/endogenous/vector_cross_product.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE VectorCrossProductVariable::get_value() const
{
    geometry::Vec value_1 = get_endogenous_parent()->get_value();
    geometry::Vec value_2 = get_other_parent()->get_value();
    return value_1.x() * value_2.y() - value_1.y() * value_2.x();
}

}
}
}
