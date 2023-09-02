
#include <ori/simcars/causal/variable_types/endogenous/vector_scalar_product.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

geometry::Vec VectorScalarProductVariable::get_value() const
{
    return get_endogenous_parent()->get_value() * get_other_parent()->get_value();
}

}
}
}
