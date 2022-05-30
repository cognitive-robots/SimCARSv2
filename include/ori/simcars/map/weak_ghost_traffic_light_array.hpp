#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/map/weak_traffic_light_array_abstract.hpp>
#include <ori/simcars/map/weak_living_traffic_light_stack_array.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template<typename T_id>
class GhostTrafficLightArray : public virtual AWeakTrafficLightArray<T_id>
{
    const std::shared_ptr<const structures::IArray<T_id>> ids;
    const std::weak_ptr<const IMap<T_id>> map;

    std::function<std::weak_ptr<const ITrafficLight<T_id>>(const std::shared_ptr<const ITrafficLight<T_id>>&)> get_weak_traffic_light_func =
            [] (const std::shared_ptr<const ITrafficLight<T_id>>& traffic_light) { return std::weak_ptr<const ITrafficLight<T_id>>(traffic_light); };

public:
    GhostTrafficLightArray(std::shared_ptr<const structures::IArray<T_id>> ids, std::shared_ptr<const IMap<T_id>> map) : ids(ids), map(map) {}

    std::shared_ptr<const IWeakTrafficLightArray<T_id>> get_self() const override
    {
        throw typename GhostTrafficLightArray<T_id>::GhostObjectException();
    }
    std::shared_ptr<const IWeakTrafficLightArray<T_id>> get_true_self() const noexcept override
    {
        std::shared_ptr<const IMap<T_id>> map = this->map.lock();

        if (map)
        {
            std::shared_ptr<const ITrafficLightArray<T_id>> traffic_lights = map->get_traffic_lights(ids);
            std::shared_ptr<WeakLivingTrafficLightStackArray<T_id>> weak_traffic_lights(
                    new WeakLivingTrafficLightStackArray<T_id>(traffic_lights->count()));
            map_array(*traffic_lights, *weak_traffic_lights, get_weak_traffic_light_func);
            return weak_traffic_lights;
        }
        else
        {
            return std::shared_ptr<const IWeakTrafficLightArray<T_id>>();
        }
    }

    size_t count() const override
    {
        throw typename GhostTrafficLightArray<T_id>::GhostObjectException();
    }
    bool contains(const std::weak_ptr<const ITrafficLight<T_id>>& val) const override
    {
        throw typename GhostTrafficLightArray<T_id>::GhostObjectException();
    }
    const std::weak_ptr<const ITrafficLight<T_id>>& operator [](size_t idx) const override
    {
        throw typename GhostTrafficLightArray<T_id>::GhostObjectException();
    }

    std::weak_ptr<const ITrafficLight<T_id>>& operator [](size_t idx) override
    {
        throw typename GhostTrafficLightArray<T_id>::GhostObjectException();
    }
};

}
}
}
