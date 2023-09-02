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
    IEndogenousVariable<P1> const *endogenous_parent_1;
    IEndogenousVariable<P2> const *endogenous_parent_2;
    IEndogenousVariable<P3> const *endogenous_parent_3;
    IVariable<P4> const *other_parent;

public:
    AQuaternaryEndogenousVariable(IEndogenousVariable<P1> const *endogenous_parent_1,
                                  IEndogenousVariable<P2> const *endogenous_parent_2,
                                  IEndogenousVariable<P3> const *endogenous_parent_3,
                                  IVariable<P4> const *other_parent) :
        endogenous_parent_1(endogenous_parent_1), endogenous_parent_2(endogenous_parent_2),
        endogenous_parent_3(endogenous_parent_3), other_parent(other_parent)
    {
        assert(endogenous_parent_1 != nullptr);
        assert(endogenous_parent_2 != nullptr);
        assert(endogenous_parent_3 != nullptr);
        assert(other_parent != nullptr);
    }

    IEndogenousVariable<P1> const* get_endogenous_parent_1() const
    {
        return endogenous_parent_1;
    }
    IEndogenousVariable<P2> const* get_endogenous_parent_2() const
    {
        return endogenous_parent_2;
    }
    IEndogenousVariable<P3> const* get_endogenous_parent_3() const
    {
        return endogenous_parent_3;
    }
    IVariable<P4> const* get_other_parent() const
    {
        return other_parent;
    }
};

}
}
}
