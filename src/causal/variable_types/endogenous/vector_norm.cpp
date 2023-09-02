
#include <ori/simcars/causal/variable_types/endogenous/vector_norm.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE VectorNormVariable::get_value() const
{
    return get_parent()->get_value().norm();
}

}
}
}
