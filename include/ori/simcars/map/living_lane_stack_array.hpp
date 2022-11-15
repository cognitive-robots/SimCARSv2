#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/map/lane_array_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class LivingLaneStackArray : public virtual ALaneArray<T_id>, public virtual structures::stl::STLStackArray<ILane<T_id> const*>
{
public:
    using structures::stl::STLStackArray<ILane<T_id> const*>::STLStackArray;

    bool is_ghost() const override
    {
        return false;
    }
    ILaneArray<T_id> const* get_true_self() const noexcept override
    {
        return this;
    }
};

}
}
}
