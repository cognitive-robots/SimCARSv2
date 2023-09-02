
#include <ori/simcars/causal/variable_types/endogenous/vector_x.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE VectorXVariable::get_value() const
{
    return get_parent()->get_value().x();
}

}
}
}
