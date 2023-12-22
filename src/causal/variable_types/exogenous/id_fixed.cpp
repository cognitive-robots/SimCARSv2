
#include <ori/simcars/causal/variable_types/exogenous/id_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

IdFixedVariable::IdFixedVariable(uint64_t value) : value(value) {}

bool IdFixedVariable::get_value(uint64_t &val) const
{
    val = value;
    return true;
}

bool IdFixedVariable::set_value(uint64_t const &val)
{
    value = val;
    return true;
}

}
}
}
