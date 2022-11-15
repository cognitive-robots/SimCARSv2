#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/map/lane_array_abstract.hpp>
#include <ori/simcars/map/living_lane_stack_array.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template<typename T_id>
class GhostLaneArray : public virtual ALaneArray<T_id>
{
    structures::IArray<T_id> const* const ids;
    IMap<T_id> const* const map;

public:
    GhostLaneArray(structures::IArray<T_id> const *ids, IMap<T_id> const *map) : ids(ids), map(map) {}

    ILaneArray<T_id> const* get_self() const override
    {
        throw typename GhostLaneArray<T_id>::GhostObjectException();
    }
    ILaneArray<T_id> const* get_true_self() const noexcept override
    {
        ILaneArray<T_id> const *true_self = map->get_lanes(ids);
        delete this;
        return true_self;
    }

    size_t count() const override
    {
        throw typename GhostLaneArray<T_id>::GhostObjectException();
    }
    bool contains(ILane<T_id> const* const &val) const override
    {
        throw typename GhostLaneArray<T_id>::GhostObjectException();
    }
    ILane<T_id> const* const& operator [](size_t idx) const override
    {
        throw typename GhostLaneArray<T_id>::GhostObjectException();
    }

    ILane<T_id> const* & operator [](size_t idx) override
    {
        throw typename GhostLaneArray<T_id>::GhostObjectException();
    }
};

}
}
}
