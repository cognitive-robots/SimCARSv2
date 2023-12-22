
#include <ori/simcars/causal/variable_types/endogenous/vector_pair_first.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorPairFirstVariable::get_value(geometry::Vec &val) const
{
    geometry::VecPair vec_pair;
    if (get_parent()->get_value(vec_pair))
    {
        val = vec_pair.first;
        return true;
    }
    else
    {
        return false;
    }
}

bool VectorPairFirstVariable::set_value(geometry::Vec const &val)
{
    geometry::VecPair vec_pair;
    if (get_parent()->get_value(vec_pair))
    {
        vec_pair.first = val;
        return get_parent()->set_value(vec_pair);
    }
    else
    {
        return false;
    }
}

}
}
}
