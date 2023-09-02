
#include <ori/simcars/causal/variable_types/endogenous/vector_pair_second.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

geometry::Vec VectorPairSecondVariable::get_value() const
{
    return get_parent()->get_value().second;
}

}
}
}
