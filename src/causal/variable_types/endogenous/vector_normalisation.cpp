
#include <ori/simcars/causal/variable_types/endogenous/vector_normalisation.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

geometry::Vec VectorNormalisationVariable::get_value() const
{
    return get_parent()->get_value().normalized();
}

}
}
}
