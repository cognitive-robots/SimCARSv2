#pragma once

#include <ori/simcars/causal/endogenous_variable_interface.hpp>

#include <cassert>

namespace ori
{
namespace simcars
{
namespace causal
{

template <typename R, typename P1, typename P2>
class ABinaryEndogenousVariable : public IEndogenousVariable<R>
{
    IEndogenousVariable<P1> *endogenous_parent;
    IVariable<P2> *other_parent;

public:
    ABinaryEndogenousVariable(IEndogenousVariable<P1> *endogenous_parent,
                              IVariable<P2> *other_parent) :
        endogenous_parent(endogenous_parent), other_parent(other_parent)
    {
        assert(endogenous_parent != nullptr);
        assert(other_parent != nullptr);
    }

    IEndogenousVariable<P1>* get_endogenous_parent() const
    {
        return endogenous_parent;
    }
    IVariable<P2>* get_other_parent() const
    {
        return other_parent;
    }
};

}
}
}
