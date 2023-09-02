
#include <ori/simcars/causal/variable_types/endogenous/o_rect_contact.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

geometry::VecPair ORectContactVariable::get_value() const
{
    return get_endogenous_parent()->get_value().calc_contact(get_other_parent()->get_value());
}

}
}
}
