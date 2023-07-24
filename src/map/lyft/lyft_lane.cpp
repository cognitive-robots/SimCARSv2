
#include <ori/simcars/map/lyft/lyft_lane.hpp>

#include <ori/simcars/geometry/trig_buff.hpp>

namespace ori
{
namespace simcars
{
namespace map
{
namespace lyft
{

LyftLane::LyftLane(uint64_t id, uint64_t left_adjacent_lane_id,
                   uint64_t right_adjacent_lane_id, structures::IArray<uint64_t> *fore_lane_ids,
                   structures::IArray<uint64_t> *aft_lane_ids, IMap const *map,
                   rapidjson::Value::ConstObject const &json_lane_data) :
    ALane(id, left_adjacent_lane_id, right_adjacent_lane_id, fore_lane_ids, aft_lane_ids, map),
    AMapObject(id, map)
{
    geometry::TrigBuff const *trig_buff = geometry::TrigBuff::get_instance();


    size_t i, j;


    rapidjson::Value::ConstArray const &left_boundary_data =
            json_lane_data["left_boundary_coord_array"].GetArray();
    size_t const left_boundary_size = left_boundary_data.Capacity();
    left_boundary = geometry::Vecs::Zero(2, left_boundary_size);
    FP_DATA_TYPE left_curvature = 0.0f;

    geometry::Vec current_link, previous_link, current_link_normalized, previous_link_normalized;
    for (i = 0; i < left_boundary_size; ++i)
    {
        left_boundary(0, i) = left_boundary_data[i][0].GetDouble();
        left_boundary(1, i) = left_boundary_data[i][1].GetDouble();
        if (i > 0)
        {
            current_link = left_boundary.col(i) - left_boundary.col(i - 1);
            current_link_normalized = current_link.normalized();
            if (i > 1)
            {
                FP_DATA_TYPE link_dot_product =
                        std::max(std::min(current_link_normalized.dot(previous_link_normalized),
                                          1.0), -1.0);
                FP_DATA_TYPE angle_mag = std::acos(link_dot_product);
                FP_DATA_TYPE angle;
                if (current_link_normalized.dot(
                            trig_buff->get_rot_mat(angle_mag) * previous_link_normalized) >=
                        current_link_normalized.dot(
                            trig_buff->get_rot_mat(-angle_mag) * previous_link_normalized))
                {
                    angle = angle_mag;
                }
                else
                {
                    angle = -angle_mag;
                }
                FP_DATA_TYPE distance_between_link_midpoints =
                        (current_link.norm() + previous_link.norm()) / 2.0f;
                FP_DATA_TYPE lane_midpoint_curvature = angle / distance_between_link_midpoints;
                left_curvature += lane_midpoint_curvature;
            }
            previous_link = current_link;
            previous_link_normalized = current_link_normalized;
        }
    }

    left_curvature /= left_boundary_size - 2;
    if (left_boundary_size < 3)
    {
        left_curvature = 0.0f;
    }

    rapidjson::Value::ConstArray const &right_boundary_data =
            json_lane_data["right_boundary_coord_array"].GetArray();
    size_t const right_boundary_size = right_boundary_data.Capacity();
    right_boundary = geometry::Vecs::Zero(2, right_boundary_size);
    FP_DATA_TYPE right_curvature = 0.0f;

    for (i = 0; i < right_boundary_size; ++i)
    {
        right_boundary(0, i) = right_boundary_data[i][0].GetDouble();
        right_boundary(1, i) = right_boundary_data[i][1].GetDouble();
        if (i > 0)
        {
            current_link = right_boundary.col(i) - right_boundary.col(i - 1);
            current_link_normalized = current_link.normalized();
            if (i > 1)
            {
                FP_DATA_TYPE link_dot_product =
                        std::max(std::min(current_link_normalized.dot(previous_link_normalized),
                                          1.0), -1.0);
                FP_DATA_TYPE angle_mag = std::acos(link_dot_product);
                FP_DATA_TYPE angle;
                if (current_link_normalized.dot(
                            trig_buff->get_rot_mat(angle_mag) * previous_link_normalized) >=
                        current_link_normalized.dot(
                            trig_buff->get_rot_mat(-angle_mag) * previous_link_normalized))
                {
                    angle = angle_mag;
                }
                else
                {
                    angle = -angle_mag;
                }
                FP_DATA_TYPE distance_between_link_midpoints =
                        (current_link.norm() + previous_link.norm()) / 2.0f;
                FP_DATA_TYPE lane_midpoint_curvature = angle / distance_between_link_midpoints;
                right_curvature += lane_midpoint_curvature;
            }
            previous_link = current_link;
            previous_link_normalized = current_link_normalized;
        }
    }

    right_curvature /= right_boundary_size - 2;
    if (right_boundary_size < 3)
    {
        right_curvature = 0.0f;
    }


    curvature = (left_curvature + right_curvature) / 2.0f;


    point_count = left_boundary_size + right_boundary_size;


    i = 0;
    j = 0;
    while (i < left_boundary_size - 1 || j < right_boundary_size - 1)
    {
        if (i < left_boundary_size - 1)
        {
            geometry::Tri tri(left_boundary.col(i), left_boundary.col(i + 1),
                              right_boundary.col(j));
            tris.push_back(tri);
            ++i;
        }

        if (j < right_boundary_size - 1)
        {
            geometry::Tri tri(right_boundary.col(j), right_boundary.col(j + 1),
                              left_boundary.col(i));
            tris.push_back(tri);
            ++j;
        }
    }


    rapidjson::Value::ConstArray const &centroid_data =
            json_lane_data["aerial_centroid"].GetArray();

    centroid(0) = centroid_data[0].GetDouble();
    centroid(1) = centroid_data[1].GetDouble();


    rapidjson::Value::ConstArray const &bounding_box_data = json_lane_data["aerial_bb"].GetArray();

    bounding_box = geometry::Rect(bounding_box_data[0].GetDouble(),
            bounding_box_data[1].GetDouble(), bounding_box_data[2].GetDouble(),
            bounding_box_data[3].GetDouble());


    access_restriction = AccessRestriction(json_lane_data["access_restriction"].GetInt());
}

geometry::Vecs const& LyftLane::get_left_boundary() const
{
    return left_boundary;
}

geometry::Vecs const& LyftLane::get_right_boundary() const
{
    return right_boundary;
}

structures::IArray<geometry::Tri> const* LyftLane::get_tris() const
{
    return &tris;
}

bool LyftLane::check_encapsulation(geometry::Vec const &point) const
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

geometry::Vec const& LyftLane::get_centroid() const
{
    return centroid;
}

size_t LyftLane::get_point_count() const
{
    return point_count;
}

geometry::Rect const& LyftLane::get_bounding_box() const
{
    return bounding_box;
}

FP_DATA_TYPE LyftLane::get_curvature() const
{
    return curvature;
}

LyftLane::AccessRestriction LyftLane::get_access_restriction() const
{
    return access_restriction;
}

}
}
}
}
