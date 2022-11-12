#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class LivingLaneStackArray : public virtual ILaneArray<T_id>, public virtual structures::stl::STLStackArray<ILane<T_id> const*>
{
public:
    using structures::stl::STLStackArray<ILane<T_id> const*>::STLStackArray;

    ILaneArray<T_id> const* get_true_self() const noexcept override
    {
        return this->shared_from_this();
    }
};

}
}
}
