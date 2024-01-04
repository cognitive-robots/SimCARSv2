
#include <ori/simcars/causal/variable_types/endogenous/vector_normalisation.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorNormalisationVariable::get_value(geometry::Vec &val) const
{
    if (get_parent()->get_value(val))
    {
        val = val.normalized();
        return true;
    }
    else
    {
        return false;
    }
}

bool VectorNormalisationVariable::set_value(geometry::Vec const &val)
{
    geometry::Vec other_val;
    if (get_parent()->get_value(other_val))
    {
        return val == other_val.normalized();
    }
    else
    {
        return true;
    }
}

}
}
}
