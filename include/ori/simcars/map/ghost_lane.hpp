#pragma once

#include <ori/simcars/map/lane_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template<typename T_id>
class GhostLane : public ALane<T_id>
{
protected:
    mutable std::shared_ptr<const GhostLane<T_id>> tether;

    GhostLane(const T_id& id, std::shared_ptr<const IMap<T_id>> map) : ALane<T_id>(id, map) {}

public:
    std::shared_ptr<const ILane<T_id>> get_self() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    void banish() const override
    {
        tether.reset();
    }
    std::shared_ptr<const ILane<T_id>> get_true_self() const noexcept override
    {
        std::shared_ptr<const ILane<T_id>> true_self = this->get_map()->get_lane(this->get_id());
        tether.reset();
        return true_self;
    }

    const geometry::Vecs& get_left_boundary() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    const geometry::Vecs& get_right_boundary() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    std::shared_ptr<const structures::IArray<geometry::Tri>> get_tris() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    bool check_encapsulation(const geometry::Vec& point) const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    const geometry::Vec& get_centroid() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    size_t get_point_count() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    const geometry::Rect& get_bounding_box() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    FP_DATA_TYPE get_mean_steer() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    GhostLane<T_id>::AccessRestriction get_access_restriction() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    std::shared_ptr<const ILane<T_id>> get_left_adjacent_lane() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    std::shared_ptr<const ILane<T_id>> get_right_adjacent_lane() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    std::shared_ptr<const ILane<T_id>> get_straight_fore_lane() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    std::shared_ptr<const ILaneArray<T_id>> get_fore_lanes() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    std::shared_ptr<const ILaneArray<T_id>> get_aft_lanes() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    std::shared_ptr<const ITrafficLightArray<T_id>> get_traffic_lights() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }

    static std::shared_ptr<const GhostLane<T_id>> spawn(const T_id& id, std::shared_ptr<const IMap<T_id>> map)
    {
        std::shared_ptr<GhostLane<T_id>> spawned_ghost_lane(new GhostLane<T_id>(id, map));
        spawned_ghost_lane->tether = spawned_ghost_lane;
        return spawned_ghost_lane;
    }
};

}
}
}
