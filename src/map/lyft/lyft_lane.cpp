
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/lyft/lyft_lane.hpp>
#include <ori/simcars/map/ghost_lane.hpp>
#include <ori/simcars/map/ghost_lane_array.hpp>
#include <ori/simcars/map/ghost_traffic_light.hpp>
#include <ori/simcars/map/ghost_traffic_light_array.hpp>

namespace ori
{
namespace simcars
{
namespace map
{
namespace lyft
{

LyftLane::LyftLane(std::string const &id, IMap<std::string> const *map, rapidjson::Value::ConstObject const &json_lane_data) : ALivingLane(id, map)
{
    geometry::TrigBuff const *trig_buff = geometry::TrigBuff::get_instance();

    rapidjson::Value::ConstArray const &left_boundary_data = json_lane_data["left_boundary_coord_array"].GetArray();
    size_t const left_boundary_size = left_boundary_data.Capacity();
    left_boundary = geometry::Vecs::Zero(2, left_boundary_size);
    FP_DATA_TYPE left_mean_steer = 0.0f;

    size_t i;
    geometry::Vec current_link, previous_link;
    for (i = 0; i < left_boundary_size; ++i)
    {
        left_boundary(0, i) = left_boundary_data[i][0].GetDouble();
        left_boundary(1, i) = left_boundary_data[i][1].GetDouble();
        if (i > 0)
        {
            current_link = left_boundary.col(i) - left_boundary.col(i - 1);
            if (i > 1)
            {
                FP_DATA_TYPE angle_mag = std::acos(current_link.dot(previous_link));
                FP_DATA_TYPE angle;
                if (current_link.dot(trig_buff->get_rot_mat(angle_mag) * previous_link) >=
                        current_link.dot(trig_buff->get_rot_mat(-angle_mag) * previous_link))
                {
                    angle = angle_mag;
                }
                else
                {
                    angle = -angle_mag;
                }
                FP_DATA_TYPE distance_between_link_midpoints = (current_link.norm() + previous_link.norm()) / 2.0f;
                FP_DATA_TYPE lane_midpoint_steer = angle / distance_between_link_midpoints;
                left_mean_steer += lane_midpoint_steer;
            }
            previous_link = current_link;
        }
    }

    left_mean_steer /= left_boundary_size - 2;
    if (left_boundary_size < 3)
    {
        left_mean_steer = 0.0f;
    }

    rapidjson::Value::ConstArray const &right_boundary_data = json_lane_data["right_boundary_coord_array"].GetArray();
    size_t const right_boundary_size = right_boundary_data.Capacity();
    right_boundary = geometry::Vecs::Zero(2, right_boundary_size);
    FP_DATA_TYPE right_mean_steer = 0.0f;

    for (i = 0; i < right_boundary_size; ++i)
    {
        right_boundary(0, i) = right_boundary_data[i][0].GetDouble();
        right_boundary(1, i) = right_boundary_data[i][1].GetDouble();
        if (i > 0)
        {
            current_link = right_boundary.col(i) - right_boundary.col(i - 1);
            if (i > 1)
            {
                FP_DATA_TYPE angle_mag = std::acos(current_link.dot(previous_link));
                FP_DATA_TYPE angle;
                if (current_link.dot(trig_buff->get_rot_mat(angle_mag) * previous_link) >=
                        current_link.dot(trig_buff->get_rot_mat(-angle_mag) * previous_link))
                {
                    angle = angle_mag;
                }
                else
                {
                    angle = -angle_mag;
                }
                FP_DATA_TYPE distance_between_link_midpoints = (current_link.norm() + previous_link.norm()) / 2.0f;
                FP_DATA_TYPE lane_midpoint_steer = angle / distance_between_link_midpoints;
                right_mean_steer += lane_midpoint_steer;
            }
            previous_link = current_link;
        }
    }

    right_mean_steer /= right_boundary_size - 2;
    if (right_boundary_size < 3)
    {
        right_mean_steer = 0.0f;
    }

    mean_steer = (left_mean_steer + right_mean_steer) / 2.0f;


    tris = new structures::stl::STLStackArray<geometry::Tri>;
    i = 0;
    size_t j = 0;
    while (i < left_boundary_size - 1 || j < right_boundary_size - 1)
    {
        if (i < left_boundary_size - 1)
        {
            geometry::Tri tri(left_boundary.col(i), left_boundary.col(i + 1), right_boundary.col(j));
            tris->push_back(tri);
            ++i;
        }

        if (j < right_boundary_size - 1)
        {
            geometry::Tri tri(right_boundary.col(j), right_boundary.col(j + 1), left_boundary.col(i));
            tris->push_back(tri);
            ++j;
        }
    }


    rapidjson::Value::ConstArray const &centroid_data = json_lane_data["aerial_centroid"].GetArray();

    centroid(0) = centroid_data[0].GetDouble();
    centroid(1) = centroid_data[1].GetDouble();


    rapidjson::Value::ConstArray const &bounding_box_data = json_lane_data["aerial_bb"].GetArray();

    bounding_box = geometry::Rect(bounding_box_data[0].GetDouble(), bounding_box_data[1].GetDouble(),
            bounding_box_data[2].GetDouble(), bounding_box_data[3].GetDouble());


    access_restriction = AccessRestriction(json_lane_data["access_restriction"].GetInt());

    std::string const left_adjacent_lane_id(json_lane_data["adjacent_left_id"].GetString(), json_lane_data["adjacent_left_id"].GetStringLength());
    if (left_adjacent_lane_id != "")
    {
        ILane *left_adjacent_lane = new GhostLane<std::string>(left_adjacent_lane_id, map);
        set_left_adjacent_lane(left_adjacent_lane);
        map->register_stray_ghost(left_adjacent_lane);
    }
    else
    {
        set_left_adjacent_lane(nullptr);
    }

    std::string const right_adjacent_lane_id(json_lane_data["adjacent_right_id"].GetString(), json_lane_data["adjacent_right_id"].GetStringLength());
    if (right_adjacent_lane_id != "")
    {
        ILane *right_adjacent_lane = new GhostLane<std::string>(right_adjacent_lane_id, map);
        set_right_adjacent_lane(right_adjacent_lane);
        map->register_stray_ghost(right_adjacent_lane);
    }
    else
    {
        set_right_adjacent_lane(nullptr);
    }


    rapidjson::Value::ConstArray const &fore_lane_data = json_lane_data["ahead_ids"].GetArray();
    size_t const fore_lane_count = fore_lane_data.Capacity();
    structures::stl::STLStackArray<std::string>* const fore_lane_ids =
            new structures::stl::STLStackArray<std::string>(fore_lane_count);

    for (i = 0; i < fore_lane_count; ++i)
    {
        (*fore_lane_ids)[i] = std::string(fore_lane_data[i].GetString(), fore_lane_data[i].GetStringLength());
    }

    ILaneArray<std::string> const* const fore_lanes = new GhostLaneArray<std::string>(fore_lane_ids, map);
    set_fore_lanes(fore_lanes);


    rapidjson::Value::ConstArray const &aft_lane_data = json_lane_data["behind_ids"].GetArray();
    size_t const aft_lane_count = aft_lane_data.Capacity();
    structures::stl::STLStackArray<std::string>* const aft_lane_ids =
            new structures::stl::STLStackArray<std::string>(aft_lane_count);

    for (i = 0; i < aft_lane_count; ++i)
    {
        (*aft_lane_ids)[i] = std::string(aft_lane_data[i].GetString(), aft_lane_data[i].GetStringLength());
    }

    ILaneArray<std::string> const* const aft_lanes = new GhostLaneArray<std::string>(aft_lane_ids, map);
    set_aft_lanes(aft_lanes);


    rapidjson::Value::ConstArray const &traffic_light_data = json_lane_data["traffic_control_ids"].GetArray();
    size_t const traffic_light_count = traffic_light_data.Capacity();
    structures::stl::STLStackArray<std::string>* const traffic_light_ids =
            new structures::stl::STLStackArray<std::string>(traffic_light_count);

    for (i = 0; i < traffic_light_count; ++i)
    {
        (*traffic_light_ids)[i] = std::string(traffic_light_data[i].GetString(), traffic_light_data[i].GetStringLength());
    }

    ITrafficLightArray<std::string> const* const traffic_lights =
            new GhostTrafficLightArray<std::string>(traffic_light_ids, map);
    set_traffic_lights(traffic_lights);
}

LyftLane::~LyftLane()
{
    delete tris;
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
    return tris;
}

bool LyftLane::check_encapsulation(geometry::Vec const &point) const
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

FP_DATA_TYPE LyftLane::get_mean_steer() const
{
    return mean_steer;
}

LyftLane::AccessRestriction LyftLane::get_access_restriction() const
{
    return access_restriction;
}

}
}
}
}
