
#include <ori/simcars/causal/variable_types/endogenous/scalar_proxy.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ScalarProxyVariable::get_value(FP_DATA_TYPE &val) const
{
    return get_parent()->get_value(val);
}

bool ScalarProxyVariable::set_value(FP_DATA_TYPE const &val)
{
    return get_parent()->set_value(val);
}

}
}
}
