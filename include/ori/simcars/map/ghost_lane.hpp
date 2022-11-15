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
public:
    GhostLane(T_id const &id, IMap<T_id> const *map) : ALane<T_id>(id, map) {}

    ILane<T_id> const* get_self() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    ILane<T_id> const* get_true_self() const noexcept override
    {
        ILane<T_id> const *true_self = this->get_map()->get_lane(this->get_id());
        delete this;
        return true_self;
    }

    geometry::Vecs const& get_left_boundary() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    geometry::Vecs const& get_right_boundary() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    structures::IArray<geometry::Tri> const* get_tris() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    bool check_encapsulation(geometry::Vec const &point) const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    geometry::Vec const& get_centroid() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    size_t get_point_count() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    geometry::Rect const& get_bounding_box() const override
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
    ILane<T_id> const* get_left_adjacent_lane() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    ILane<T_id> const* get_right_adjacent_lane() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    ILane<T_id> const* get_straight_fore_lane() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    ILaneArray<T_id> const* get_fore_lanes() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    ILaneArray<T_id> const* get_aft_lanes() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
    ITrafficLightArray<T_id> const* get_traffic_lights() const override
    {
        throw typename GhostLane<T_id>::GhostObjectException();
    }
};

}
}
}
