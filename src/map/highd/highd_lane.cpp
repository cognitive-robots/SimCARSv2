
#include <ori/simcars/map/ghost_lane.hpp>
#include <ori/simcars/map/ghost_lane_array.hpp>
#include <ori/simcars/map/ghost_traffic_light_array.hpp>
#include <ori/simcars/map/highd/highd_lane.hpp>

namespace ori
{
namespace simcars
{
namespace map
{
namespace highd
{

HighDLane::HighDLane(uint8_t id, IMap<uint8_t> const *map, FP_DATA_TYPE upper_bound, FP_DATA_TYPE lower_bound,
                     bool driving_left, uint8_t left_adjacent_lane_id, uint8_t right_adjacent_lane_id) :
    ALivingLane(id, map), point_count(4), mean_steer(0.0f), access_restriction(HighDLane::AccessRestriction::NO_RESTRICTION)
{
    if (driving_left)
    {
        left_boundary = geometry::Vecs::Zero(2, 2);
        left_boundary(0, 0) = 460.0f;
        left_boundary(1, 0) = lower_bound;
        left_boundary(0, 1) = -40.0f;
        left_boundary(1, 1) = lower_bound;

        right_boundary = geometry::Vecs::Zero(2, 2);
        right_boundary(0, 0) = 460.0f;
        right_boundary(1, 0) = upper_bound;
        right_boundary(0, 1) = -40.0f;
        right_boundary(1, 1) = upper_bound;
    }
    else
    {
        left_boundary = geometry::Vecs::Zero(2, 2);
        left_boundary(0, 0) = -40.0f;
        left_boundary(1, 0) = upper_bound;
        left_boundary(0, 1) = 460.0f;
        left_boundary(1, 1) = upper_bound;

        right_boundary = geometry::Vecs::Zero(2, 2);
        right_boundary(0, 0) = -40.0f;
        right_boundary(1, 0) = lower_bound;
        right_boundary(0, 1) = 460.0f;
        right_boundary(1, 1) = lower_bound;
    }

    centroid(0) = 210.0f;
    centroid(1) = (upper_bound + lower_bound) / 2.0f;

    tris = new structures::stl::STLStackArray<geometry::Tri>(2);
    geometry::Tri tri_1(left_boundary.col(1), left_boundary.col(0), right_boundary.col(0));
    tris->push_back(tri_1);
    geometry::Tri tri_2(right_boundary.col(0), right_boundary.col(1), left_boundary.col(1));
    tris->push_back(tri_2);

    bounding_box = geometry::Rect(-40.0f, upper_bound, 460.0f, lower_bound);

    if (left_adjacent_lane_id != 0)
    {
        GhostLane<uint8_t> *left_adjacent_lane = new GhostLane<uint8_t>(left_adjacent_lane_id, map);
        this->set_left_adjacent_lane(left_adjacent_lane);
        map->register_stray_ghost(left_adjacent_lane);
    }

    if (right_adjacent_lane_id != 0)
    {
        GhostLane<uint8_t> *right_adjacent_lane = new GhostLane<uint8_t>(right_adjacent_lane_id, map);
        this->set_right_adjacent_lane(right_adjacent_lane);
        map->register_stray_ghost(right_adjacent_lane);
    }
}

HighDLane::~HighDLane()
{
    delete tris;
}

geometry::Vecs const& HighDLane::get_left_boundary() const
{
    return left_boundary;
}

geometry::Vecs const& HighDLane::get_right_boundary() const
{
    return right_boundary;
}

structures::IArray<geometry::Tri> const* HighDLane::get_tris() const
{
    return tris;
}

bool HighDLane::check_encapsulation(geometry::Vec const &point) const
{
    if (bounding_box.check_encapsulation(point))
    {
        size_t i;
        for (i = 0; i < tris->count(); ++i)
        {
            if ((*tris)[i].check_encapsulation(point))
            {
                return true;
            }
        }
    }
    return false;
}

geometry::Vec const& HighDLane::get_centroid() const
{
    return centroid;
}

size_t HighDLane::get_point_count() const
{
    return point_count;
}

geometry::Rect const& HighDLane::get_bounding_box() const
{
    return bounding_box;
}

FP_DATA_TYPE HighDLane::get_mean_steer() const
{
    return mean_steer;
}

HighDLane::AccessRestriction HighDLane::get_access_restriction() const
{
    return access_restriction;
}

}
}
}
}
