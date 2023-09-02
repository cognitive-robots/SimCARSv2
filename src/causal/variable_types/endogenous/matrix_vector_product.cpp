
#include <ori/simcars/causal/variable_types/endogenous/matrix_vector_product.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

geometry::Vec MatrixVectorProductVariable::get_value() const
{
    return get_endogenous_parent()->get_value() * get_other_parent()->get_value();
}

}
}
}
