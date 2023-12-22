
#include <ori/simcars/causal/variable_types/endogenous/o_rect_collision.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ORectCollisionVariable::get_value(bool &val) const
{
    geometry::ORect rect_1, rect_2;
    if (get_endogenous_parent()->get_value(rect_1) && get_other_parent()->get_value(rect_2))
    {
        val = rect_1.check_collision(rect_2);
        return true;
    }
    else
    {
        return false;
    }
}

bool ORectCollisionVariable::set_value(bool const &val)
{
    geometry::ORect rect_1, rect_2;
    if (get_endogenous_parent()->get_value(rect_1) && get_other_parent()->get_value(rect_2))
    {
        return val == rect_1.check_collision(rect_2);
    }
    else
    {
        return false;
    }
}

}
}
}
