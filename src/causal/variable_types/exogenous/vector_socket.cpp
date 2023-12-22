
#include <ori/simcars/causal/variable_types/exogenous/vector_socket.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

VectorSocketVariable::VectorSocketVariable(geometry::Vec default_value,
                                           IVariable<geometry::Vec> *parent) :
    default_value(default_value), parent(parent) {}


bool VectorSocketVariable::get_value(geometry::Vec &val) const
{
    if (parent != nullptr)
    {
        return parent->get_value(val);
    }
    else
    {
        val = default_value;
        return true;
    }
}

IVariable<geometry::Vec> const* VectorSocketVariable::get_parent() const
{
    return parent;
}

bool VectorSocketVariable::set_value(geometry::Vec const &val)
{
    if (parent != nullptr)
    {
        return parent->set_value(val);
    }
    else
    {
        default_value = val;
        return true;
    }
}

void VectorSocketVariable::set_parent(IVariable<geometry::Vec> *parent)
{
    this->parent = parent;
}

}
}
}
