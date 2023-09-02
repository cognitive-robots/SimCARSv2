
#include <ori/simcars/causal/variable_types/endogenous/o_rect_construction.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

geometry::ORect ORectConstructionVariable::get_value() const
{
    return geometry::ORect(get_endogenous_parent_1()->get_value(),
                           get_endogenous_parent_2()->get_value(),
                           get_endogenous_parent_3()->get_value(),
                           get_other_parent()->get_value());
}

}
}
}
