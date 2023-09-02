
#include <ori/simcars/causal/variable_types/endogenous/vector_dot_product.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE VectorDotProductVariable::get_value() const
{
    return get_endogenous_parent()->get_value().dot(get_other_parent()->get_value());
}

}
}
}
