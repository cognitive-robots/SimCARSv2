
#include <ori/simcars/map/highd/highd_lane.hpp>

namespace ori
{
namespace simcars
{
namespace map
{
namespace highd
{

HighDLane::HighDLane(uint64_t id, uint64_t left_adjacent_lane_id, uint64_t right_adjacent_lane_id,
                     IDrivingMap const *map, FP_DATA_TYPE upper_bound, FP_DATA_TYPE lower_bound,
                     bool driving_left) :
    ALane(id, left_adjacent_lane_id, right_adjacent_lane_id,
          new structures::stl::STLStackArray<uint64_t>,
          new structures::stl::STLStackArray<uint64_t>, map), ADrivingMapObject(id, map),
    curvature(0.0f), access_restriction(HighDLane::AccessRestriction::NO_RESTRICTION)
{
    if (driving_left)
    {
        left_boundary = geometry::Vecs::Zero(2, 2);
        //left_boundary(0, 0) = 460.0;
        left_boundary(0, 0) = 560.0;
        left_boundary(1, 0) = lower_bound;
        //left_boundary(0, 1) = -40.0;
        left_boundary(0, 1) = -140.0;
        left_boundary(1, 1) = lower_bound;

        right_boundary = geometry::Vecs::Zero(2, 2);
        //right_boundary(0, 0) = 460.0;
        right_boundary(0, 0) = 560.0;
        right_boundary(1, 0) = upper_bound;
        //right_boundary(0, 1) = -40.0;
        right_boundary(0, 1) = -140.0;
        right_boundary(1, 1) = upper_bound;

        waypoints = geometry::Vecs::Zero(2, 2);
        //waypoints(0, 0) = 460.0;
        waypoints(0, 0) = 560.0;
        waypoints(1, 0) = 0.5 * (upper_bound + lower_bound);
        //waypoints(0, 1) = -40.0;
        waypoints(0, 1) = -140.0;
        waypoints(1, 1) = 0.5 * (upper_bound + lower_bound);

    }
    else
    {
        left_boundary = geometry::Vecs::Zero(2, 2);
        //left_boundary(0, 0) = -40.0;
        left_boundary(0, 0) = -140.0;
        left_boundary(1, 0) = upper_bound;
        //left_boundary(0, 1) = 460.0;
        left_boundary(0, 1) = 560.0;
        left_boundary(1, 1) = upper_bound;

        right_boundary = geometry::Vecs::Zero(2, 2);
        //right_boundary(0, 0) = -40.0;
        right_boundary(0, 0) = -140.0;
        right_boundary(1, 0) = lower_bound;
        //right_boundary(0, 1) = 460.0;
        right_boundary(0, 1) = 560.0;
        right_boundary(1, 1) = lower_bound;

        waypoints = geometry::Vecs::Zero(2, 2);
        //waypoints(0, 0) = -40.0;
        waypoints(0, 0) = -140.0;
        waypoints(1, 0) = 0.5 * (upper_bound + lower_bound);
        //waypoints(0, 1) = 460.0;
        waypoints(0, 1) = 560.0;
        waypoints(1, 1) = 0.5 * (upper_bound + lower_bound);
    }

    centroid(0) = 210.0f;
    centroid(1) = (upper_bound + lower_bound) / 2.0f;

    geometry::Tri tri_1(left_boundary.col(1), left_boundary.col(0), right_boundary.col(0));
    tris.push_back(tri_1);
    geometry::Tri tri_2(right_boundary.col(0), right_boundary.col(1), left_boundary.col(1));
    tris.push_back(tri_2);

    //bounding_box = geometry::Rect(-40.0f, upper_bound, 460.0f, lower_bound);
    bounding_box = geometry::Rect(-140.0f, upper_bound, 560.0f, lower_bound);
}

geometry::Vecs const& HighDLane::get_left_boundary() const
{
    return left_boundary;
}

geometry::Vecs const& HighDLane::get_right_boundary() const
{
    return right_boundary;
}

geometry::Vecs const& HighDLane::get_waypoints() const
{
    return waypoints;
}

structures::IArray<geometry::Tri> const* HighDLane::get_tris() const
{
    return &tris;
}

bool HighDLane::check_encapsulation(geometry::Vec const &point) const
{
    if (bounding_box.check_encapsulation(point))
    {
        size_t i;
        for (i = 0; i < tris.count(); ++i)
        {
            if (tris[i].check_encapsulation(point))
            {
                return true;
            }
        }
    }
    return false;
}

geometry::Vec HighDLane::map_point(geometry::Vec const &point) const
{
    geometry::Vec lane_dir = (waypoints.col(1) - waypoints.col(0)).normalized();
    geometry::Vec to_point = point - waypoints.col(0);
    FP_DATA_TYPE lane_dist = lane_dir.dot(to_point);
    return waypoints.col(0) + lane_dir * lane_dist;
}

geometry::Vec const& HighDLane::get_centroid() const
{
    return centroid;
}

geometry::Rect const& HighDLane::get_bounding_box() const
{
    return bounding_box;
}

FP_DATA_TYPE HighDLane::get_curvature() const
{
    return curvature;
}

HighDLane::AccessRestriction HighDLane::get_access_restriction() const
{
    return access_restriction;
}

}
}
}
}
