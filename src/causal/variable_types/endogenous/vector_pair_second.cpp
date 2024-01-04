
#include <ori/simcars/causal/variable_types/endogenous/vector_pair_second.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorPairSecondVariable::get_value(geometry::Vec &val) const
{
    geometry::VecPair vec_pair;
    if (get_parent()->get_value(vec_pair))
    {
        val = vec_pair.second;
        return true;
    }
    else
    {
        return false;
    }
}

bool VectorPairSecondVariable::set_value(geometry::Vec const &val)
{
    geometry::VecPair vec_pair;
    if (get_parent()->get_value(vec_pair))
    {
        vec_pair.second = val;
        return get_parent()->set_value(vec_pair);
    }
    else
    {
        return true;
    }
}

}
}
}
