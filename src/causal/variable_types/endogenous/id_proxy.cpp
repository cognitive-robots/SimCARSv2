
#include <ori/simcars/causal/variable_types/endogenous/id_proxy.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

uint64_t IdProxyVariable::get_value() const
{
    return get_parent()->get_value();
}

}
}
}
