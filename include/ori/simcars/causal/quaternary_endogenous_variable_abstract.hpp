#pragma once

#include <ori/simcars/causal/endogenous_variable_interface.hpp>

#include <cassert>

namespace ori
{
namespace simcars
{
namespace causal
{

template <typename R, typename P1, typename P2, typename P3, typename P4>
class AQuaternaryEndogenousVariable : public IEndogenousVariable<R>
{
    IEndogenousVariable<P1> *endogenous_parent_1;
    IEndogenousVariable<P2> *endogenous_parent_2;
    IEndogenousVariable<P3> *endogenous_parent_3;
    IVariable<P4> *other_parent;

public:
    AQuaternaryEndogenousVariable(IEndogenousVariable<P1> *endogenous_parent_1,
                                  IEndogenousVariable<P2> *endogenous_parent_2,
                                  IEndogenousVariable<P3> *endogenous_parent_3,
                                  IVariable<P4> *other_parent) :
        endogenous_parent_1(endogenous_parent_1), endogenous_parent_2(endogenous_parent_2),
        endogenous_parent_3(endogenous_parent_3), other_parent(other_parent)
    {
        assert(endogenous_parent_1 != nullptr);
        assert(endogenous_parent_2 != nullptr);
        assert(endogenous_parent_3 != nullptr);
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
    IEndogenousVariable<P3>* get_endogenous_parent_3() const
    {
        return endogenous_parent_3;
    }
    IVariable<P4>* get_other_parent() const
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
    void set_endogenous_parent_3(IEndogenousVariable<P3> *parent)
    {
        this->endogenous_parent_3 = parent;
    }
    void set_other_parent(IVariable<P4> *parent)
    {
        this->other_parent = parent;
    }
};

}
}
}
