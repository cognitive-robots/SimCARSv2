#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/map/weak_lane_array_abstract.hpp>
#include <ori/simcars/map/weak_living_lane_stack_array.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template<typename T_id>
class GhostLaneArray : public virtual AWeakLaneArray<T_id>
{
    const std::shared_ptr<const structures::IArray<T_id>> ids;
    const std::weak_ptr<const IMap<T_id>> map;

    std::function<std::weak_ptr<const ILane<T_id>>(const std::shared_ptr<const ILane<T_id>>&)> get_weak_lane_func =
            [] (const std::shared_ptr<const ILane<T_id>>& traffic_light) { return std::weak_ptr<const ILane<T_id>>(traffic_light); };

public:
    GhostLaneArray(std::shared_ptr<const structures::IArray<T_id>> ids, std::shared_ptr<const IMap<T_id>> map) : ids(ids), map(map) {}

    std::shared_ptr<const IWeakLaneArray<T_id>> get_self() const override
    {
        throw typename GhostLaneArray<T_id>::GhostObjectException();
    }
    std::shared_ptr<const IWeakLaneArray<T_id>> get_true_self() const noexcept override
    {
        std::shared_ptr<const IMap<T_id>> map = this->map.lock();

        if (map)
        {
            std::shared_ptr<const ILaneArray<T_id>> lanes = map->get_lanes(ids);
            std::shared_ptr<WeakLivingLaneStackArray<T_id>> weak_lanes(
                    new WeakLivingLaneStackArray<T_id>(lanes->count()));
            map_array(*lanes, *weak_lanes, get_weak_lane_func);
            return weak_lanes;
        }
        else
        {
            return std::shared_ptr<const IWeakLaneArray<T_id>>();
        }
    }

    size_t count() const override
    {
        throw typename GhostLaneArray<T_id>::GhostObjectException();
    }
    bool contains(const std::weak_ptr<const ILane<T_id>>& val) const override
    {
        throw typename GhostLaneArray<T_id>::GhostObjectException();
    }
    const std::weak_ptr<const ILane<T_id>>& operator [](size_t idx) const override
    {
        throw typename GhostLaneArray<T_id>::GhostObjectException();
    }

    std::weak_ptr<const ILane<T_id>>& operator [](size_t idx) override
    {
        throw typename GhostLaneArray<T_id>::GhostObjectException();
    }
};

}
}
}
