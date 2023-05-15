#pragma once

#include <ori/simcars/map/living_lane_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{
namespace plg
{

class PLGLane : public ALivingLane<uint8_t>
{
    geometry::Vecs left_boundary, right_boundary;
    geometry::Vec centroid;
    structures::IStackArray<geometry::Tri> *tris;
    size_t point_count;
    FP_DATA_TYPE mean_steer;
    geometry::Rect bounding_box;
    AccessRestriction access_restriction;

public:
    PLGLane(uint8_t id, IMap<uint8_t> const *map, geometry::Vecs *vertices);
    ~PLGLane() override;

    geometry::Vecs const& get_left_boundary() const override;
    geometry::Vecs const& get_right_boundary() const override;
    structures::IArray<geometry::Tri> const* get_tris() const override;
    bool check_encapsulation(geometry::Vec const &point) const override;
    geometry::Vec const& get_centroid() const override;
    size_t get_point_count() const override;
    geometry::Rect const& get_bounding_box() const override;
    FP_DATA_TYPE get_mean_steer() const override;
    ILane::AccessRestriction get_access_restriction() const override;
};


}
}
}
}
