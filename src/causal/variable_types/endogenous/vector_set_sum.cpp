
#include <ori/simcars/causal/variable_types/endogenous/vector_set_sum.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorSetSumVariable::get_value(geometry::Vec &val) const
{
    structures::IArray<IEndogenousVariable<geometry::Vec>*> const *parents = get_array();
    geometry::Vec current_val, total_val = geometry::Vec::Zero();
    for (size_t i = 0; i < count(); ++i)
    {
        if ((*parents)[i]->get_value(current_val))
        {
            total_val += current_val;
        }
        else
        {
            return false;
        }
    }

    val = total_val;
    return true;
}

bool VectorSetSumVariable::set_value(geometry::Vec const &val)
{
    structures::IArray<IEndogenousVariable<geometry::Vec>*> const *parents = get_array();
    geometry::Vec current_val, total_val = geometry::Vec::Zero();
    for (size_t i = 0; i < count(); ++i)
    {
        if ((*parents)[i]->get_value(current_val))
        {
            total_val += current_val;
        }
        else
        {
            // TODO: Potentially look at a better way of handling this, e.g. propogating remaining sum to undefined variable
            return true;
        }
    }

    return val == total_val;
}

}
}
}
