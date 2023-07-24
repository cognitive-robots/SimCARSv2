#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/map/lane_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{
namespace plg
{

class PLGLane : public virtual ALane
{
    geometry::Vecs left_boundary, right_boundary;
    structures::stl::STLStackArray<geometry::Tri> tris;
    geometry::Vec centroid;
    size_t point_count;
    geometry::Rect bounding_box;
    FP_DATA_TYPE curvature;
    AccessRestriction access_restriction;

public:
    PLGLane(uint64_t id, IMap const *map, geometry::Vecs const *vertices);

    geometry::Vecs const& get_left_boundary() const override;
    geometry::Vecs const& get_right_boundary() const override;
    structures::IArray<geometry::Tri> const* get_tris() const override;
    bool check_encapsulation(geometry::Vec const &point) const override;
    geometry::Vec const& get_centroid() const override;
    size_t get_point_count() const override;
    geometry::Rect const& get_bounding_box() const override;
    FP_DATA_TYPE get_curvature() const override;
    ILane::AccessRestriction get_access_restriction() const override;
};


}
}
}
}
