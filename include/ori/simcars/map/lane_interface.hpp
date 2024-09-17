#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/geometry/tri.hpp>
#include <ori/simcars/geometry/rect.hpp>
#include <ori/simcars/map/declarations.hpp>
#include <ori/simcars/map/driving_map_object_interface.hpp>
#include <ori/simcars/map/lane_branch.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

class ILane : public virtual IDrivingMapObject
{
public:
    enum class AccessRestriction
    {
        UNKNOWN = -1,
        NO_RESTRICTION = 0,
        ONLY_HOV = 1,
        ONLY_BUS = 2,
        ONLY_BIKE = 3,
        ONLY_TURN = 4
    };

    virtual geometry::Vecs const& get_left_boundary() const = 0;
    virtual geometry::Vecs const& get_right_boundary() const = 0;
    virtual geometry::Vecs const& get_waypoints() const = 0;
    virtual structures::IArray<geometry::Tri> const* get_tris() const = 0;
    virtual bool check_encapsulation(geometry::Vec const &point) const = 0;
    virtual geometry::Vec map_point(geometry::Vec const &point) const = 0;
    virtual geometry::Vec const& get_centroid() const = 0;
    virtual geometry::Rect const& get_bounding_box() const = 0;
    virtual FP_DATA_TYPE get_curvature() const = 0;
    virtual AccessRestriction get_access_restriction() const = 0;
    virtual ILane const* get_left_adjacent_lane() const = 0;
    virtual ILane const* get_right_adjacent_lane() const = 0;
    virtual LaneBranch const* get_fore_lane_branch() const = 0;
    virtual LaneBranch const* get_aft_lane_branch() const = 0;
};

}
}
}
