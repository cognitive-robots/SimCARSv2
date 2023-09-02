#pragma once

#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/causal/endogenous_variable_interface.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

template <typename R, typename P>
class ASetEndogenousVariable : public structures::stl::STLSet<IEndogenousVariable<P> const*>,
        public IEndogenousVariable<R>
{
public:
    ASetEndogenousVariable(std::initializer_list<IEndogenousVariable<P> const*> init_list) :
        structures::stl::STLSet<IEndogenousVariable<P> const*>(init_list)
    {
        assert(this->count() > 0);
        structures::IArray<IEndogenousVariable<P> const*> const *parents = this->get_array();
        for (size_t i = 0; i < this->count(); ++i)
        {
            assert((*parents)[i] != nullptr);
        }
    }
};

}
}
}
