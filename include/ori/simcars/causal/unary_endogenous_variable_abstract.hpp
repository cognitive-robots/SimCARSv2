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
    IVariable<P1> const *parent;

public:
    AUnaryEndogenousVariable(IVariable<P1> const *parent) : parent(parent)
    {
        assert(parent != nullptr);
    }

    IVariable<P1> const* get_parent() const
    {
        return parent;
    }
};

}
}
}
