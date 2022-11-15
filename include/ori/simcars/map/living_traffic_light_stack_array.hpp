#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/map/traffic_light_array_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class LivingTrafficLightStackArray : public virtual ATrafficLightArray<T_id>, public virtual structures::stl::STLStackArray<ITrafficLight<T_id> const*>
{
public:
    using structures::stl::STLStackArray<ITrafficLight<T_id> const*>::STLStackArray;

    bool is_ghost() const override
    {
        return false;
    }
    ITrafficLightArray<T_id> const* get_true_self() const noexcept override
    {
        return this;
    }
};

}
}
}
