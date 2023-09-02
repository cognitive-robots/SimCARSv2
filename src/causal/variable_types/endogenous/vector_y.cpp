
#include <ori/simcars/causal/variable_types/endogenous/vector_y.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE VectorYVariable::get_value() const
{
    return get_parent()->get_value().x();
}

}
}
}
