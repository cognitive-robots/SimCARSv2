#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/geometry/tri.hpp>
#include <ori/simcars/geometry/rect.hpp>
#include <ori/simcars/map/soul_interface.hpp>
#include <ori/simcars/map/declarations.hpp>
#include <ori/simcars/map/map_object_interface.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class ILane : public virtual IMapObject<T_id>, public virtual ISoul<ILane<T_id>>
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
    virtual structures::IArray<geometry::Tri> const* get_tris() const = 0;
    virtual bool check_encapsulation(geometry::Vec const &point) const = 0;
    virtual geometry::Vec const& get_centroid() const = 0;
    virtual size_t get_point_count() const = 0;
    virtual geometry::Rect const& get_bounding_box() const = 0;
    virtual FP_DATA_TYPE get_mean_steer() const = 0;
    virtual ILane<T_id>::AccessRestriction get_access_restriction() const = 0;
    virtual ILane<T_id> const* get_left_adjacent_lane() const = 0;
    virtual ILane<T_id> const* get_right_adjacent_lane() const = 0;
    virtual ILane<T_id> const* get_straight_fore_lane() const = 0;
    virtual ILaneArray<T_id> const* get_fore_lanes() const = 0;
    virtual ILaneArray<T_id> const* get_aft_lanes() const = 0;
    virtual ITrafficLightArray<T_id> const* get_traffic_lights() const = 0;
};

}
}
}
