#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/map/lane_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{
namespace highd
{

class HighDLane : public virtual ALane
{
    geometry::Vecs left_boundary, right_boundary, waypoints;
    structures::stl::STLStackArray<geometry::Tri> tris;
    geometry::Vec centroid;
    geometry::Rect bounding_box;
    FP_DATA_TYPE curvature;
    AccessRestriction access_restriction;

public:
    HighDLane(uint64_t id, uint64_t left_adjacent_lane_id, uint64_t right_adjacent_lane_id,
              IDrivingMap const *map, FP_DATA_TYPE upper_bound, FP_DATA_TYPE lower_bound,
              bool driving_left);

    geometry::Vecs const& get_left_boundary() const override;
    geometry::Vecs const& get_right_boundary() const override;
    geometry::Vecs const& get_waypoints() const override;
    structures::IArray<geometry::Tri> const* get_tris() const override;
    bool check_encapsulation(geometry::Vec const &point) const override;
    geometry::Vec map_point(geometry::Vec const &point) const override;
    geometry::Vec const& get_centroid() const override;
    geometry::Rect const& get_bounding_box() const override;
    FP_DATA_TYPE get_curvature() const override;
    ILane::AccessRestriction get_access_restriction() const override;
};


}
}
}
}
