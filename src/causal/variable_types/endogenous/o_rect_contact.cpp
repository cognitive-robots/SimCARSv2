
#include <ori/simcars/causal/variable_types/endogenous/o_rect_contact.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ORectContactVariable::get_value(geometry::VecPair &val) const
{
    geometry::ORect rect_1, rect_2;
    if (get_endogenous_parent()->get_value(rect_1) && get_other_parent()->get_value(rect_2))
    {
        val = rect_1.calc_contact(rect_2);
        return true;
    }
    else
    {
        return false;
    }
}

bool ORectContactVariable::set_value(geometry::VecPair const &val)
{
    geometry::ORect rect_1, rect_2;
    if (get_endogenous_parent()->get_value(rect_1) && get_other_parent()->get_value(rect_2))
    {
        return val == rect_1.calc_contact(rect_2);
    }
    else
    {
        return false;
    }
}

}
}
}
