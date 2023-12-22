
#include <ori/simcars/causal/variable_types/endogenous/vector_conditional.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorConditionalVariable::get_value(geometry::Vec &val) const
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

bool VectorConditionalVariable::set_value(geometry::Vec const &val)
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
