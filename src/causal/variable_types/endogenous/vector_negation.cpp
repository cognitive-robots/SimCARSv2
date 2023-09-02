
#include <ori/simcars/causal/variable_types/endogenous/vector_negation.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

geometry::Vec VectorNegationVariable::get_value() const
{
    return -get_parent()->get_value();
}

}
}
}
