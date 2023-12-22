
#include <ori/simcars/causal/variable_types/endogenous/vector_proxy.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool VectorProxyVariable::get_value(geometry::Vec &val) const
{
    return get_parent()->get_value(val);
}

bool VectorProxyVariable::set_value(geometry::Vec const &val)
{
    return get_parent()->set_value(val);
}

}
}
}
