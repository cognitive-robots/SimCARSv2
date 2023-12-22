
#include <ori/simcars/causal/variable_types/endogenous/scalar_conditional.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ScalarConditionalVariable::get_value(FP_DATA_TYPE &val) const
{
    bool condition;
    if (get_other_parent()->get_value(condition))
    {
        if (condition)
        {
            return get_endogenous_parent_1()->get_value(val);
        }
        else
        {
            return get_endogenous_parent_2()->get_value(val);
        }
    }
    else
    {
        return false;
    }
}

bool ScalarConditionalVariable::set_value(FP_DATA_TYPE const &val)
{
    // TODO: Allow set to change condition variable
    bool condition;
    if (get_other_parent()->get_value(condition))
    {
        if (condition)
        {
            return get_endogenous_parent_1()->set_value(val);
        }
        else
        {
            return get_endogenous_parent_2()->set_value(val);
        }
    }
    else
    {
        return false;
    }
}

}
}
}
