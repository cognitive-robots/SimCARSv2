#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class LivingTrafficLightStackArray : public virtual ITrafficLightArray<T_id>, public virtual structures::stl::STLStackArray<ITrafficLight<T_id> const*>
{
public:
    using structures::stl::STLStackArray<ITrafficLight<T_id> const*>::STLStackArray;

    ITrafficLightArray<T_id> const* get_true_self() const noexcept override
    {
        return this->shared_from_this();
    }
};

}
}
}
