
#include <ori/simcars/causal/variable_types/endogenous/scalar_min.hpp>

#include <algorithm>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ScalarMinVariable::get_value(FP_DATA_TYPE &val) const
{
    FP_DATA_TYPE other_val;
    if (get_other_parent()->get_value(other_val) && get_endogenous_parent()->get_value(val))
    {
        val = std::min(val, other_val);
        return true;
    }
    else
    {
        return false;
    }
}

bool ScalarMinVariable::set_value(FP_DATA_TYPE const &val)
{
    FP_DATA_TYPE val_1, val_2;
    if (get_endogenous_parent()->get_value(val_1))
    {
        if (get_other_parent()->get_value(val_2))
        {
            if (val_1 <= val_2)
            {
                if (val < val_1)
                {
                    return false;
                }
                else
                {
                    return get_endogenous_parent()->set_value(val);
                }
            }
            else
            {
                if (val < val_2)
                {
                    return false;
                }
                else
                {
                    return get_other_parent()->set_value(val);
                }
            }
        }
        else
        {
            if (val < val_1)
            {
                return false;
            }
            else
            {
                return get_endogenous_parent()->set_value(val);
            }
        }
    }
    else
    {
        if (get_other_parent()->get_value(val_2))
        {
            if (val < val_2)
            {
                return false;
            }
            else
            {
                return get_other_parent()->set_value(val);
            }
        }
        else
        {
            return true;
        }
    }
}

}
}
}
