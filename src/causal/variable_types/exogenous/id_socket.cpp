
#include <ori/simcars/causal/variable_types/exogenous/id_socket.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

IdSocketVariable::IdSocketVariable(uint64_t default_value, IVariable<uint64_t> *parent) :
    default_value(default_value), parent(parent) {}

bool IdSocketVariable::get_value(uint64_t &val) const
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

IVariable<uint64_t> const* IdSocketVariable::get_parent() const
{
    return parent;
}

bool IdSocketVariable::set_value(uint64_t const &val)
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

void IdSocketVariable::set_parent(IVariable<uint64_t> *parent)
{
    this->parent = parent;
}

}
}
}
