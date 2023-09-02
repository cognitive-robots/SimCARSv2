
#include <ori/simcars/causal/variable_types/exogenous/id_socket.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

IdSocketVariable::IdSocketVariable(uint64_t default_value, IVariable<uint64_t> const *parent) :
    default_value(default_value), parent(parent) {}

uint64_t IdSocketVariable::get_value() const
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

IVariable<uint64_t> const* IdSocketVariable::get_parent() const
{
    return parent;
}

void IdSocketVariable::set_parent(const IVariable<uint64_t> *parent)
{
    this->parent = parent;
}

}
}
}
