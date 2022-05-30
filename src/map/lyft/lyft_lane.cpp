
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/map/lyft/lyft_lane.hpp>
#include <ori/simcars/map/ghost_lane.hpp>
#include <ori/simcars/map/weak_ghost_lane_array.hpp>
#include <ori/simcars/map/ghost_traffic_light.hpp>
#include <ori/simcars/map/weak_ghost_traffic_light_array.hpp>

namespace ori
{
namespace simcars
{
namespace map
{
namespace lyft
{

LyftLane::LyftLane(const std::string& id, std::shared_ptr<const IMap<std::string>> map, const rapidjson::Value::ConstObject& json_lane_data) : ALivingLane(id, map)
{
    const rapidjson::Value::ConstArray left_boundary_data = json_lane_data["left_boundary_coord_array"].GetArray();
    const size_t left_boundary_size = left_boundary_data.Capacity();
    left_boundary = geometry::Vecs::Zero(2, left_boundary_size);

    size_t i;
    for (i = 0; i < left_boundary_size; ++i)
    {
        left_boundary(0, i) = left_boundary_data[i][0].GetDouble();
        left_boundary(1, i) = left_boundary_data[i][1].GetDouble();
    }


    const rapidjson::Value::ConstArray right_boundary_data = json_lane_data["right_boundary_coord_array"].GetArray();
    const size_t right_boundary_size = right_boundary_data.Capacity();
    right_boundary = geometry::Vecs::Zero(2, right_boundary_size);

    for (i = 0; i < right_boundary_size; ++i)
    {
        right_boundary(0, i) = right_boundary_data[i][0].GetDouble();
        right_boundary(1, i) = right_boundary_data[i][1].GetDouble();
    }


    tris.reset(new structures::stl::STLStackArray<geometry::Tri>);
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


    const rapidjson::Value::ConstArray centroid_data = json_lane_data["aerial_centroid"].GetArray();

    centroid(0) = centroid_data[0].GetDouble();
    centroid(1) = centroid_data[1].GetDouble();


    const rapidjson::Value::ConstArray bounding_box_data = json_lane_data["aerial_bb"].GetArray();

    bounding_box = geometry::Rect(bounding_box_data[0].GetDouble(), bounding_box_data[1].GetDouble(),
            bounding_box_data[2].GetDouble(), bounding_box_data[3].GetDouble());


    access_restriction = AccessRestriction(json_lane_data["access_restriction"].GetInt());


    const std::string left_adjacent_lane_id(json_lane_data["adjacent_left_id"].GetString(), json_lane_data["adjacent_left_id"].GetStringLength());
    set_left_adjacent_lane(GhostLane<std::string>::spawn(left_adjacent_lane_id, map));


    const std::string right_adjacent_lane_id(json_lane_data["adjacent_right_id"].GetString(), json_lane_data["adjacent_right_id"].GetStringLength());
    set_right_adjacent_lane(GhostLane<std::string>::spawn(right_adjacent_lane_id, map));


    const rapidjson::Value::ConstArray fore_lane_data = json_lane_data["ahead_ids"].GetArray();
    const size_t fore_lane_count = fore_lane_data.Capacity();
    const std::shared_ptr<structures::stl::STLStackArray<std::string>> fore_lane_ids(
                new structures::stl::STLStackArray<std::string>(fore_lane_count));

    for (i = 0; i < fore_lane_count; ++i)
    {
        (*fore_lane_ids)[i] = std::string(fore_lane_data[i].GetString(), fore_lane_data[i].GetStringLength());
    }

    const std::shared_ptr<const IWeakLaneArray<std::string>> fore_lanes(new GhostLaneArray<std::string>(fore_lane_ids, map));
    set_fore_lanes(fore_lanes);


    const rapidjson::Value::ConstArray aft_lane_data = json_lane_data["behind_ids"].GetArray();
    const size_t aft_lane_count = aft_lane_data.Capacity();
    const std::shared_ptr<structures::stl::STLStackArray<std::string>> aft_lane_ids(
            new structures::stl::STLStackArray<std::string>(aft_lane_count));

    for (i = 0; i < aft_lane_count; ++i)
    {
        (*aft_lane_ids)[i] = std::string(aft_lane_data[i].GetString(), aft_lane_data[i].GetStringLength());
    }

    const std::shared_ptr<const IWeakLaneArray<std::string>> aft_lanes(new GhostLaneArray<std::string>(aft_lane_ids, map));
    set_aft_lanes(aft_lanes);


    const rapidjson::Value::ConstArray traffic_light_data = json_lane_data["traffic_control_ids"].GetArray();
    const size_t traffic_light_count = traffic_light_data.Capacity();
    const std::shared_ptr<structures::stl::STLStackArray<std::string>> traffic_light_ids(
                new structures::stl::STLStackArray<std::string>(traffic_light_count));

    for (i = 0; i < traffic_light_count; ++i)
    {
        (*traffic_light_ids)[i] = std::string(traffic_light_data[i].GetString(), traffic_light_data[i].GetStringLength());
    }

    const std::shared_ptr<const IWeakTrafficLightArray<std::string>> traffic_lights(
                new GhostTrafficLightArray<std::string>(traffic_light_ids, map));
    set_traffic_lights(traffic_lights);
}

const geometry::Vecs& LyftLane::get_left_boundary() const
{
    return left_boundary;
}

const geometry::Vecs& LyftLane::get_right_boundary() const
{
    return right_boundary;
}

std::shared_ptr<const structures::IArray<geometry::Tri>> LyftLane::get_tris() const
{
    return tris;
}

bool LyftLane::check_encapsulation(const geometry::Vec& point) const
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

const geometry::Vec& LyftLane::get_centroid() const
{
    return centroid;
}

size_t LyftLane::get_point_count() const
{
    return point_count;
}

const geometry::Rect& LyftLane::get_bounding_box() const
{
    return bounding_box;
}

LyftLane::AccessRestriction LyftLane::get_access_restriction() const
{
    return access_restriction;
}

}
}
}
}
