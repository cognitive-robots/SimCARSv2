
#include <ori/simcars/map/plg/plg_lane.hpp>

#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>

#define LANE_DILATION 3.65

namespace ori
{
namespace simcars
{
namespace map
{
namespace plg
{

PLGLane::PLGLane(uint64_t id, IMap const *map, geometry::Vecs const *vertices) :
    ALane(id, 0, 0, new structures::stl::STLStackArray<uint64_t>,
          new structures::stl::STLStackArray<uint64_t>, map), AMapObject(id, map),
    access_restriction(PLGLane::AccessRestriction::NO_RESTRICTION)
{
    geometry::TrigBuff const *trig_buff = geometry::TrigBuff::get_instance();

    left_boundary = geometry::Vecs::Zero(2, vertices->cols());
    right_boundary = geometry::Vecs::Zero(2, vertices->cols());
    waypoints = geometry::Vecs::Zero(2, vertices->cols());

    centroid = geometry::Vec::Zero();

    FP_DATA_TYPE min_x = (*vertices)(0, 0);
    FP_DATA_TYPE max_x = (*vertices)(0, 0);
    FP_DATA_TYPE min_y = (*vertices)(1, 0);
    FP_DATA_TYPE max_y = (*vertices)(1, 0);

    curvature = 0.0f;

    geometry::Vec previous_translation;
    geometry::Vec previous_vertex_diff;
    size_t i;
    for (i = 0; i < vertices->cols(); ++i)
    {
        geometry::Vec const current_vertex = vertices->col(i);

        waypoints.col(i) = current_vertex;

        if (i < vertices->cols() - 1)
        {
            geometry::Vec const next_vertex = vertices->col(i + 1);
            geometry::Vec const current_vertex_diff = next_vertex - current_vertex;
            geometry::RotMat rot_mat;
            rot_mat << 0.0f, -1.0f, 1.0f, 0.0f;
            geometry::Vec const vertex_diff_tang = rot_mat * current_vertex_diff;
            geometry::Vec current_translation =
                    (LANE_DILATION / 2.0f) * vertex_diff_tang.normalized();

            if (i == 0)
            {
                left_boundary.col(i) = current_vertex + current_translation;
                right_boundary.col(i) = current_vertex - current_translation;
            }
            else
            {
                geometry::Vec const l1 = left_boundary.col(i - 1);
                geometry::Vec const l2 = current_vertex + previous_translation;
                geometry::Vec const l3 = current_vertex + current_translation;
                geometry::Vec const l4 = next_vertex + current_translation;

                FP_DATA_TYPE l_common_denom =
                        (l1.x() - l2.x()) * (l3.y() - l4.y()) -
                        (l1.y() - l2.y()) * (l3.x() - l4.x());

                if (l_common_denom == 0.0f)
                {
                    left_boundary.col(i) = current_vertex + current_translation;
                    right_boundary.col(i) = current_vertex - current_translation;
                }
                else
                {
                    FP_DATA_TYPE l_x_numer =
                            (l1.x() * l2.y() - l1.y() * l2.x()) * (l3.x() - l4.x()) -
                            (l1.x() - l2.x()) * (l3.x() * l4.y() - l3.y() * l4.x());
                    FP_DATA_TYPE l_y_numer =
                            (l1.x() * l2.y() - l1.y() * l2.x()) * (l3.y() - l4.y()) -
                            (l1.y() - l2.y()) * (l3.x() * l4.y() - l3.y() * l4.x());

                    left_boundary(0, i) = l_x_numer / l_common_denom;
                    left_boundary(1, i) = l_y_numer / l_common_denom;

                    geometry::Vec const r1 = right_boundary.col(i - 1);
                    geometry::Vec const r2 = current_vertex - previous_translation;
                    geometry::Vec const r3 = current_vertex - current_translation;
                    geometry::Vec const r4 = next_vertex - current_translation;

                    FP_DATA_TYPE r_common_denom =
                            (r1.x() - r2.x()) * (r3.y() - r4.y()) -
                            (r1.y() - r2.y()) * (r3.x() - r4.x());
                    FP_DATA_TYPE r_x_numer =
                            (r1.x() * r2.y() - r1.y() * r2.x()) * (r3.x() - r4.x()) -
                            (r1.x() - r2.x()) * (r3.x() * r4.y() - r3.y() * r4.x());
                    FP_DATA_TYPE r_y_numer =
                            (r1.x() * r2.y() - r1.y() * r2.x()) * (r3.y() - r4.y()) -
                            (r1.y() - r2.y()) * (r3.x() * r4.y() - r3.y() * r4.x());

                    right_boundary(0, i) = r_x_numer / r_common_denom;
                    right_boundary(1, i) = r_y_numer / r_common_denom;

                    geometry::Vec const current_vertex_diff_normalised =
                            current_vertex_diff.normalized();
                    geometry::Vec const previous_vertex_diff_normalised =
                            previous_vertex_diff.normalized();
                    FP_DATA_TYPE vertex_diff_dot_prod =
                            std::max(std::min(
                                         previous_vertex_diff_normalised.dot(
                                             current_vertex_diff_normalised),
                                         1.0), -1.0);
                    FP_DATA_TYPE angle_mag = std::acos(vertex_diff_dot_prod);
                    FP_DATA_TYPE angle;
                    if (current_vertex_diff_normalised.dot(
                                trig_buff->get_rot_mat(angle_mag) *
                                previous_vertex_diff_normalised) >=
                            current_vertex_diff_normalised.dot(
                                trig_buff->get_rot_mat(-angle_mag) *
                                previous_vertex_diff_normalised))
                    {
                        angle = angle_mag;
                    }
                    else
                    {
                        angle = -angle_mag;
                    }
                    FP_DATA_TYPE distance_between_link_midpoints =
                            (current_vertex_diff.norm() + previous_vertex_diff.norm()) /
                            2.0f;
                    FP_DATA_TYPE lane_midpoint_curvature = angle /
                            distance_between_link_midpoints;
                    curvature += lane_midpoint_curvature;
                }
            }

            previous_translation = current_translation;
            previous_vertex_diff = current_vertex_diff;
        }
        else
        {
            left_boundary.col(i) = current_vertex + previous_translation;
            right_boundary.col(i) = current_vertex - previous_translation;
        }

        centroid += left_boundary.col(i);
        centroid += right_boundary.col(i);

        min_x = std::min(std::min(left_boundary(0, i), right_boundary(0, i)), min_x);
        max_x = std::max(std::max(left_boundary(0, i), right_boundary(0, i)), max_x);
        min_y = std::min(std::min(left_boundary(1, i), right_boundary(1, i)), min_y);
        max_y = std::max(std::max(left_boundary(1, i), right_boundary(1, i)), max_y);
    }

    centroid /= vertices->cols() * 2.0f;

    curvature /= vertices->cols() - 2.0f;

    bounding_box = geometry::Rect(min_x, min_y, max_x, max_y);

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
}

geometry::Vecs const& PLGLane::get_left_boundary() const
{
    return left_boundary;
}

geometry::Vecs const& PLGLane::get_right_boundary() const
{
    return right_boundary;
}

geometry::Vecs const& PLGLane::get_waypoints() const
{
    return waypoints;
}

structures::IArray<geometry::Tri> const* PLGLane::get_tris() const
{
    return &tris;
}

bool PLGLane::check_encapsulation(geometry::Vec const &point) const
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

geometry::Vec PLGLane::map_point(geometry::Vec const &point) const
{
    size_t min_dist_index = 0;
    FP_DATA_TYPE min_dist = (point - waypoints.col(0)).norm();

    size_t i;
    for (i = 1; i < waypoints.cols() - 1; ++i)
    {
        FP_DATA_TYPE current_dist = (point - waypoints.col(i)).norm();
        if (current_dist < min_dist)
        {
            min_dist_index = i;
            min_dist = current_dist;
        }
    }

    geometry::Vec min_waypoint = waypoints.col(min_dist_index);
    geometry::Vec next_waypoint = waypoints.col(min_dist_index + 1);
    geometry::Vec lane_dir = (next_waypoint - min_waypoint).normalized();
    geometry::Vec to_point = point - min_waypoint;
    FP_DATA_TYPE lane_dist = lane_dir.dot(to_point);

    return min_waypoint + lane_dir * lane_dist;
}

geometry::Vec const& PLGLane::get_centroid() const
{
    return centroid;
}

geometry::Rect const& PLGLane::get_bounding_box() const
{
    return bounding_box;
}

FP_DATA_TYPE PLGLane::get_curvature() const
{
    return curvature;
}

PLGLane::AccessRestriction PLGLane::get_access_restriction() const
{
    return access_restriction;
}

}
}
}
}
