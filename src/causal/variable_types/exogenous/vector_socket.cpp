
#include <ori/simcars/causal/variable_types/exogenous/vector_socket.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

VectorSocketVariable::VectorSocketVariable(geometry::Vec default_value,
                                           IVariable<geometry::Vec> const *parent) :
    default_value(default_value), parent(parent) {}

geometry::Vec VectorSocketVariable::get_value() const
{
    if (parent != nullptr)
    {
        return parent->get_value();
    }
    else
    {
        return default_value;
    }
}

IVariable<geometry::Vec> const* VectorSocketVariable::get_parent() const
{
    return parent;
}

void VectorSocketVariable::set_parent(const IVariable<geometry::Vec> *parent)
{
    this->parent = parent;
}

}
}
}
