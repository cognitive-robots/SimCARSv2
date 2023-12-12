
#include <ori/simcars/causal/variable_types/exogenous/id_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

IdFixedVariable::IdFixedVariable(uint64_t value) : value(value) {}

uint64_t IdFixedVariable::get_value() const
{
    return value;
}

}
}
}
