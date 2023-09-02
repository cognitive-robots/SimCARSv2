
#include <ori/simcars/causal/variable_types/endogenous/vector_proxy.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

geometry::Vec VectorProxyVariable::get_value() const
{
    return get_parent()->get_value();
}

}
}
}
