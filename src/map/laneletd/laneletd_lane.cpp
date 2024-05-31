
#include <ori/simcars/map/laneletd/laneletd_lane.hpp>

#include <ori/simcars/geometry/trig_buff.hpp>

namespace ori
{
namespace simcars
{
namespace map
{
namespace laneletd
{

LaneletDLane::LaneletDLane(uint64_t id, uint64_t left_adjacent_lane_id,
                           uint64_t right_adjacent_lane_id,
                           structures::IArray<uint64_t> *fore_lanes,
                           structures::IArray<uint64_t> *aft_lanes,
                           IMap const *map, lanelet::ConstLanelet &lanelet,
                           geometry::Vec const &utm_origin) :
    ALane(id, left_adjacent_lane_id, right_adjacent_lane_id, fore_lanes, aft_lanes, map),
    AMapObject(id, map)
{
    geometry::TrigBuff const *trig_buff = geometry::TrigBuff::get_instance();

    centroid = geometry::Vec::Zero();

    FP_DATA_TYPE min_x = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_x = std::numeric_limits<FP_DATA_TYPE>::min();
    FP_DATA_TYPE min_y = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_y = std::numeric_limits<FP_DATA_TYPE>::min();

    lanelet::ConstLineString2d left_boundary_line_string = lanelet.leftBound2d();
    left_boundary = geometry::Vecs::Zero(2, left_boundary_line_string.size());
    size_t i;
    for (i = 0; i < left_boundary_line_string.size(); ++i)
    {
        geometry::Vec const point = left_boundary_line_string[i].basicPoint() - utm_origin;
        left_boundary.col(i) = point;
        centroid += point;
        min_x = std::min(point.x(), min_x);
        max_x = std::max(point.x(), max_x);
        min_y = std::min(point.y(), min_y);
        max_y = std::max(point.y(), max_y);
    }

    lanelet::ConstLineString2d right_boundary_line_string = lanelet.rightBound2d();
    right_boundary = geometry::Vecs::Zero(2, right_boundary_line_string.size());
    for (i = 0; i < right_boundary_line_string.size(); ++i)
    {
        geometry::Vec const point = right_boundary_line_string[i].basicPoint() - utm_origin;
        right_boundary.col(i) = point;
        centroid += point;
        min_x = std::min(point.x(), min_x);
        max_x = std::max(point.x(), max_x);
        min_y = std::min(point.y(), min_y);
        max_y = std::max(point.y(), max_y);
    }

    centroid /= (left_boundary_line_string.size() + right_boundary_line_string.size());

    bounding_box = geometry::Rect(min_x, min_y, max_x, max_y);

    curvature = 0.0f;

    lanelet::ConstLineString2d centre_line_string = lanelet.centerline2d();
    waypoints = geometry::Vecs::Zero(2, centre_line_string.size());
    geometry::Vec previous_diff;
    for (i = 0; i < centre_line_string.size(); ++i)
    {
        waypoints.col(i) = centre_line_string[i].basicPoint() - utm_origin;

        if (i > 0)
        {
            geometry::Vec const current_diff = waypoints.col(i) - waypoints.col(i - 1);

            if (i > 1)
            {
                geometry::Vec const current_diff_normalised = current_diff.normalized();
                geometry::Vec const previous_diff_normalised = previous_diff.normalized();
                FP_DATA_TYPE diff_dot_prod = std::max(std::min(previous_diff_normalised.dot(
                                                                   current_diff_normalised),
                                                               1.0), -1.0);
                FP_DATA_TYPE angle_mag = std::acos(diff_dot_prod);
                FP_DATA_TYPE angle;
                if (current_diff_normalised.dot(trig_buff->get_rot_mat(angle_mag) *
                                                previous_diff_normalised) >=
                        current_diff_normalised.dot(trig_buff->get_rot_mat(-angle_mag) *
                                                    previous_diff_normalised))
                {
                    angle = angle_mag;
                }
                else
                {
                    angle = -angle_mag;
                }
                FP_DATA_TYPE distance_between_midpoints =
                        (current_diff.norm() + previous_diff.norm()) / 2.0f;
                FP_DATA_TYPE lane_midpoint_curvature = angle / distance_between_midpoints;
                curvature += lane_midpoint_curvature;
            }

            previous_diff = current_diff;
        }
    }

    curvature /= waypoints.cols() - 2.0f;

    i = 0;
    size_t j = 0;
    while (i < left_boundary.cols() - 1 || j < right_boundary.cols() - 1)
    {
        if (i < left_boundary.cols() - 1)
        {
            geometry::Tri tri(left_boundary.col(i), left_boundary.col(i + 1),
                              right_boundary.col(j));
            tris.push_back(tri);
            ++i;
        }

        if (j < right_boundary.cols() - 1)
        {
            geometry::Tri tri(right_boundary.col(j), right_boundary.col(j + 1),
                              left_boundary.col(i));
            tris.push_back(tri);
            ++j;
        }
    }

    // TODO: Extract access restrictions from the lanelet regulatory elements
    access_restriction = LaneletDLane::AccessRestriction::NO_RESTRICTION;
}

geometry::Vecs const& LaneletDLane::get_left_boundary() const
{
    return left_boundary;
}

geometry::Vecs const& LaneletDLane::get_right_boundary() const
{
    return right_boundary;
}

geometry::Vecs const& LaneletDLane::get_waypoints() const
{
    return waypoints;
}

structures::IArray<geometry::Tri> const* LaneletDLane::get_tris() const
{
    return &tris;
}

bool LaneletDLane::check_encapsulation(geometry::Vec const &point) const
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

geometry::Vec LaneletDLane::map_point(geometry::Vec const &point) const
{
    for (size_t i = 0; i < waypoints.cols() - 1; ++i)
    {
        geometry::Vec waypoint_diff = waypoints.col(i + 1) - waypoints.col(i);
        geometry::Vec lane_dir = waypoint_diff.normalized();
        geometry::Vec to_point = point - waypoints.col(i);
        FP_DATA_TYPE lane_dist = lane_dir.dot(to_point);
        if (lane_dist < 0)
        {
            return waypoints.col(i);
        }
        else if (lane_dist < waypoint_diff.norm() || i == waypoints.cols() - 2)
        {
            return waypoints.col(i) + lane_dir * lane_dist;
        }
    }

    if (waypoints.cols() > 0)
    {
        return waypoints.col(waypoints.cols() - 1);
    }
    else
    {
        return point;
    }
}

geometry::Vec const& LaneletDLane::get_centroid() const
{
    return centroid;
}

geometry::Rect const& LaneletDLane::get_bounding_box() const
{
    return bounding_box;
}

FP_DATA_TYPE LaneletDLane::get_curvature() const
{
    return curvature;
}

LaneletDLane::AccessRestriction LaneletDLane::get_access_restriction() const
{
    return access_restriction;
}

}
}
}
}
