#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/map/weak_traffic_light_array_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class WeakLivingTrafficLightStackArray : public virtual AWeakTrafficLightArray<T_id>, public virtual structures::stl::STLStackArray<std::weak_ptr<const ITrafficLight<T_id>>>
{
public:
    using structures::stl::STLStackArray<std::weak_ptr<const ITrafficLight<T_id>>>::STLStackArray;

    std::shared_ptr<const IWeakTrafficLightArray<T_id>> get_true_self() const noexcept override
    {
        return std::shared_ptr<const IWeakTrafficLightArray<T_id>>(this);
    }
};

}
}
}
