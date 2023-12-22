
#include <ori/simcars/causal/variable_types/endogenous/vector_sum.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorSumVariable::get_value(geometry::Vec &val) const
{
    geometry::Vec other_val;
    if (get_other_parent()->get_value(other_val) && get_endogenous_parent()->get_value(val))
    {
        val = val + other_val;
        return true;
    }
    else
    {
        return false;
    }
}

bool VectorSumVariable::set_value(geometry::Vec const &val)
{
    geometry::Vec val_1, val_2;
    if (get_endogenous_parent()->get_value(val_1))
    {
        if (get_other_parent()->get_value(val_2))
        {
            return val == val_1 + val_2;
        }
        else
        {
            return get_other_parent()->set_value(val - val_1);
        }
    }
    else
    {
        if (get_other_parent()->get_value(val_2))
        {
            return get_endogenous_parent()->set_value(val - val_2);
        }
        else
        {
            return false;
        }
    }
}

}
}
}
