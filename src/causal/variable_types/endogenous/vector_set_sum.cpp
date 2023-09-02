
#include <ori/simcars/causal/variable_types/endogenous/vector_set_sum.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

geometry::Vec VectorSetSumVariable::get_value() const
{
    structures::IArray<IEndogenousVariable<geometry::Vec> const*> const *parents = get_array();
    geometry::Vec value;
    for (size_t i = 0; i < count(); ++i)
    {
        value += (*parents)[i]->get_value();
    }
    return value;
}

}
}
}
