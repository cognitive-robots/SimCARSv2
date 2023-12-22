
#include <ori/simcars/causal/variable_types/endogenous/id_proxy.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool IdProxyVariable::get_value(uint64_t &val) const
{
    return get_parent()->get_value(val);
}

bool IdProxyVariable::set_value(uint64_t const &val)
{
    return get_parent()->set_value(val);
}

}
}
}
