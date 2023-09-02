
#include <ori/simcars/causal/variable_types/endogenous/o_rect_collision.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ORectCollisionVariable::get_value() const
{
    return get_endogenous_parent()->get_value().check_collision(get_other_parent()->get_value());
}

}
}
}
