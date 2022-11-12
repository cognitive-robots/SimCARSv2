#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/map/living_traffic_light_stack_array.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template<typename T_id>
class GhostTrafficLightArray : public virtual ITrafficLightArray<T_id>
{
    structures::IArray<T_id> const* const ids;
    IMap<T_id> const* const map;

public:
    GhostTrafficLightArray(structures::IArray<T_id> const *ids, IMap<T_id> const *map) : ids(ids), map(map) {}

    ITrafficLightArray<T_id> const* get_self() const override
    {
        throw typename GhostTrafficLightArray<T_id>::GhostObjectException();
    }
    ITrafficLightArray<T_id> const* get_true_self() const noexcept override
    {
        if (map)
        {
            return map->get_traffic_lights(ids);
        }
        else
        {
            return nullptr;
        }
    }

    size_t count() const override
    {
        throw typename GhostTrafficLightArray<T_id>::GhostObjectException();
    }
    bool contains(ITrafficLight<T_id> const* const &val) const override
    {
        throw typename GhostTrafficLightArray<T_id>::GhostObjectException();
    }
    ITrafficLight<T_id> const* const& operator [](size_t idx) const override
    {
        throw typename GhostTrafficLightArray<T_id>::GhostObjectException();
    }

    ITrafficLight<T_id> const* & operator [](size_t idx) override
    {
        throw typename GhostTrafficLightArray<T_id>::GhostObjectException();
    }
};

}
}
}
