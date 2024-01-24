#pragma once

#include <ori/simcars/causal/endogenous_variable_interface.hpp>

#include <cassert>

namespace ori
{
namespace simcars
{
namespace causal
{

template <typename R, typename P1, typename P2, typename P3>
class ATernaryEndogenousVariable : public IEndogenousVariable<R>
{
    IEndogenousVariable<P1> *endogenous_parent_1;
    IEndogenousVariable<P2> *endogenous_parent_2;
    IVariable<P3> *other_parent;

public:
    ATernaryEndogenousVariable(IEndogenousVariable<P1> *endogenous_parent_1,
                               IEndogenousVariable<P2> *endogenous_parent_2,
                               IVariable<P3> *other_parent) :
        endogenous_parent_1(endogenous_parent_1), endogenous_parent_2(endogenous_parent_2),
        other_parent(other_parent)
    {
        assert(endogenous_parent_1 != nullptr);
        assert(endogenous_parent_2 != nullptr);
        assert(other_parent != nullptr);
    }

    IEndogenousVariable<P1>* get_endogenous_parent_1() const
    {
        return endogenous_parent_1;
    }
    IEndogenousVariable<P2>* get_endogenous_parent_2() const
    {
        return endogenous_parent_2;
    }
    IVariable<P3>* get_other_parent() const
    {
        return other_parent;
    }

    void set_endogenous_parent_1(IEndogenousVariable<P1> *parent)
    {
        this->endogenous_parent_1 = parent;
    }
    void set_endogenous_parent_2(IEndogenousVariable<P2> *parent)
    {
        this->endogenous_parent_2 = parent;
    }
    void set_other_parent(IVariable<P3> *parent)
    {
        this->other_parent = parent;
    }
};

}
}
}
