
#include <ori/simcars/causal/variable_types/endogenous/scalar_proxy.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE ScalarProxyVariable::get_value() const
{
    return get_parent()->get_value();
}

}
}
}
