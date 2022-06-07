#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/map/weak_lane_array_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class WeakLivingLaneStackArray : public virtual AWeakLaneArray<T_id>, public virtual structures::stl::STLStackArray<std::weak_ptr<const ILane<T_id>>>, public std::enable_shared_from_this<WeakLivingLaneStackArray<T_id>>
{
public:
    using structures::stl::STLStackArray<std::weak_ptr<const ILane<T_id>>>::STLStackArray;

    std::shared_ptr<const IWeakLaneArray<T_id>> get_true_self() const noexcept override
    {
        return this->shared_from_this();
    }
};

}
}
}
