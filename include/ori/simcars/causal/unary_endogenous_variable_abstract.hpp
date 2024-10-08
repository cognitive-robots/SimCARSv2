#pragma once

#include <ori/simcars/causal/endogenous_variable_interface.hpp>

#include <cassert>

namespace ori
{
namespace simcars
{
namespace causal
{

template <typename R, typename P1>
class AUnaryEndogenousVariable : public IEndogenousVariable<R>
{
    IVariable<P1> *parent;

public:
    AUnaryEndogenousVariable(IVariable<P1> *parent) : parent(parent)
    {
        assert(parent != nullptr);
    }

    IVariable<P1>* get_parent() const
    {
        return parent;
    }

    void set_parent(IVariable<P1> *parent)
    {
        this->parent = parent;
    }
};

}
}
}
