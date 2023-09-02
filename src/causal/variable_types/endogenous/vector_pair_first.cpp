
#include <ori/simcars/causal/variable_types/endogenous/vector_pair_first.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

geometry::Vec VectorPairFirstVariable::get_value() const
{
    return get_parent()->get_value().first;
}

}
}
}
