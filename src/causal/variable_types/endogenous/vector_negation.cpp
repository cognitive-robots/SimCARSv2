
#include <ori/simcars/causal/variable_types/endogenous/vector_negation.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorNegationVariable::get_value(geometry::Vec &val) const
{
    if (get_parent()->get_value(val))
    {
        val = -val;
        return true;
    }
    else
    {
        return false;
    }
}

bool VectorNegationVariable::set_value(geometry::Vec const &val)
{
    return get_parent()->set_value(-val);
}

}
}
}
