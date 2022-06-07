
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/map/lyft/lyft_map.hpp>

#include <laudrup/lz4_stream/lz4_stream.hpp>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

#include <exception>

namespace ori
{
namespace simcars
{
namespace map
{
namespace lyft
{

void LyftMap::save_virt(std::ofstream& output_filestream) const
{
    throw utils::NotImplementedException();
}

void LyftMap::load_virt(std::ifstream& input_filestream)
{
    lz4_stream::istream input_lz4_stream(input_filestream);
    rapidjson::BasicIStreamWrapper input_lz4_json_stream(input_lz4_stream);

    rapidjson::Document json_document;
    json_document.ParseStream(input_lz4_json_stream);

    id_to_lane_dict.reset(new structures::stl::STLDictionary<std::string, std::shared_ptr<LyftLane>>());
    id_to_traffic_light_dict.reset(new structures::stl::STLDictionary<std::string, std::shared_ptr<LyftTrafficLight>>());
    map_grid_dict.reset(new geometry::GridDictionary<MapGridRect<std::string>>(geometry::Vec(0, 0), 100));

    std::shared_ptr<structures::IDictionary<std::string, std::shared_ptr<LyftTrafficLight::IFaceDictionary>>> id_to_face_colour_to_face_type_dict(
                new structures::stl::STLDictionary<std::string, std::shared_ptr<LyftTrafficLight::IFaceDictionary>>);
    std::shared_ptr<structures::IDictionary<std::string, std::shared_ptr<LyftTrafficLight::TemporalStateDictionary>>> id_to_timestamp_to_state_dict(
                new structures::stl::STLDictionary<std::string, std::shared_ptr<LyftTrafficLight::TemporalStateDictionary>>);

    rapidjson::Value::Object traffic_light_face_id_to_traffic_light_id =
            json_document["id_traffic_light_face_id_traffic_light_dict"].GetObject();
    rapidjson::Value::Object traffic_light_face_id_to_state_data =
            json_document["id_traffic_light_face_frames_dict"].GetObject();
    rapidjson::Value::Object traffic_light_face_id_to_traffic_light_face_data =
            json_document["id_traffic_light_face_dict"].GetObject();

    enum class LyftTrafficLightFaceState
    {
        UNKNOWN = -1,
        INACTIVE = 0,
        ACTIVE = 1
    };

    for (rapidjson::Value::ConstMemberIterator traffic_light_face_id_to_traffic_light_face_itr =
         traffic_light_face_id_to_traffic_light_face_data.MemberBegin();
         traffic_light_face_id_to_traffic_light_face_itr !=
         traffic_light_face_id_to_traffic_light_face_data.MemberEnd();
         ++traffic_light_face_id_to_traffic_light_face_itr)
    {
        const std::string& traffic_light_face_id =
                traffic_light_face_id_to_traffic_light_face_itr->name.GetString();
        if (!traffic_light_face_id_to_traffic_light_id.HasMember(traffic_light_face_id.c_str()))
        {
            continue;
        }
        const std::string& traffic_light_id =
                traffic_light_face_id_to_traffic_light_id[traffic_light_face_id.c_str()].GetString();

        std::shared_ptr<LyftTrafficLight::IFaceDictionary> face_colour_to_face_type_dict;
        if (id_to_face_colour_to_face_type_dict->contains(traffic_light_id))
        {
            face_colour_to_face_type_dict = (*id_to_face_colour_to_face_type_dict)[traffic_light_id];
        }
        else
        {
            face_colour_to_face_type_dict.reset(
                        new structures::stl::STLDictionary<LyftTrafficLight::FaceColour, LyftTrafficLight::FaceType>());
            id_to_face_colour_to_face_type_dict->update(traffic_light_id, face_colour_to_face_type_dict);
        }

        rapidjson::Value::ConstObject traffic_light_face_data =
                traffic_light_face_id_to_traffic_light_face_itr->value.GetObject();
        const std::string& traffic_light_face_type_str = traffic_light_face_data["traffic_light_face_type"].GetString();

        LyftTrafficLight::FaceColour face_colour = LyftTrafficLight::FaceColour::UNKNOWN;
        LyftTrafficLight::FaceType face_type = LyftTrafficLight::FaceType::UNKNOWN;

        std::string token;
        size_t current_delim, next_delim;
        for (current_delim = 0, next_delim = traffic_light_face_type_str.find('_');
             current_delim == next_delim;
             current_delim = next_delim, next_delim = traffic_light_face_type_str.find('_', current_delim))
        {
            token = traffic_light_face_type_str.substr(current_delim, next_delim - current_delim);

            if (token == "signal")
            {
                face_type = LyftTrafficLight::FaceType::STANDARD;
            }
            else if (token == "red")
            {
                face_colour = LyftTrafficLight::FaceColour::RED;
            }
            else if (token == "yellow")
            {
                face_colour = LyftTrafficLight::FaceColour::YELLOW;
            }
            else if (token == "green")
            {
                face_colour = LyftTrafficLight::FaceColour::GREEN;
            }
            else if (token == "arrow")
            {
                face_type = LyftTrafficLight::FaceType(static_cast<int>(face_type) | static_cast<int>(LyftTrafficLight::FaceType::ARROW));
            }
            else if (token == "left")
            {
                face_type = LyftTrafficLight::FaceType(static_cast<int>(face_type) | static_cast<int>(LyftTrafficLight::FaceType::LEFT));
            }
            else if (token == "right")
            {
                face_type = LyftTrafficLight::FaceType(static_cast<int>(face_type) | static_cast<int>(LyftTrafficLight::FaceType::RIGHT));
            }
            else if (token == "upper")
            {
                face_type = LyftTrafficLight::FaceType(static_cast<int>(face_type) | static_cast<int>(LyftTrafficLight::FaceType::UPPER));
            }
            else if (token == "u")
            {
                current_delim = next_delim;
                next_delim = traffic_light_face_type_str.find('_', current_delim);
                token = traffic_light_face_type_str.substr(current_delim, next_delim - current_delim);

                if (token == "turn")
                {
                    face_type = LyftTrafficLight::FaceType(static_cast<int>(face_type) | static_cast<int>(LyftTrafficLight::FaceType::UTURN));
                }
                else
                {
                    face_type = LyftTrafficLight::FaceType::UNKNOWN;
                }
            }
            else if (token == "flashing")
            {
                face_type = LyftTrafficLight::FaceType(static_cast<int>(face_type) | static_cast<int>(LyftTrafficLight::FaceType::FLASHING));
            }
            else
            {
                face_type = LyftTrafficLight::FaceType::UNKNOWN;
            }
        }

        face_colour_to_face_type_dict->update(face_colour, face_type);


        std::shared_ptr<LyftTrafficLight::TemporalStateDictionary> timestamp_to_state_dict;
        if (id_to_timestamp_to_state_dict->contains(traffic_light_id))
        {
            timestamp_to_state_dict = (*id_to_timestamp_to_state_dict)[traffic_light_id];
        }
        else
        {
            timestamp_to_state_dict.reset(
                    new LyftTrafficLight::TemporalStateDictionary(temporal::Duration(50), 10));
            id_to_timestamp_to_state_dict->update(traffic_light_id, timestamp_to_state_dict);
        }

        if (traffic_light_face_id_to_state_data.HasMember(traffic_light_face_id.c_str()))
        {
            rapidjson::Value::Array state_data = traffic_light_face_id_to_state_data[traffic_light_face_id.c_str()].GetArray();
            for (const rapidjson::Value& state_data_element : state_data)
            {
                rapidjson::Value::ConstArray state_pair = state_data_element.GetArray();
                temporal::Time timestamp(temporal::Duration(state_pair[0].GetInt64()));
                LyftTrafficLightFaceState face_state = static_cast<LyftTrafficLightFaceState>(state_pair[1].GetInt());

                if (face_state == LyftTrafficLightFaceState::ACTIVE)
                {
                    timestamp_to_state_dict->update(
                                timestamp,
                                std::shared_ptr<LyftTrafficLight::State>(
                                    new LyftTrafficLight::State(face_colour)));
                }
            }
        }
    }

    rapidjson::Value::Object lane_id_to_lane_data = json_document["id_lane_dict"].GetObject();

    for (rapidjson::Value::ConstMemberIterator lane_id_to_lane_itr = lane_id_to_lane_data.MemberBegin();
         lane_id_to_lane_itr != lane_id_to_lane_data.MemberEnd(); ++lane_id_to_lane_itr)
    {
        const std::string& lane_id = lane_id_to_lane_itr->name.GetString();
        std::shared_ptr<LyftLane> lane(new LyftLane(lane_id, shared_from_this(), lane_id_to_lane_itr->value.GetObject()));
        id_to_lane_dict->update(lane_id, lane);
        const geometry::Rect& lane_bounding_box = lane->get_bounding_box();
        map_grid_dict->chebyshev_proliferate(
            lane_bounding_box.get_origin(),
            lane_bounding_box.get_width() / 2.0f,
            lane_bounding_box.get_height() / 2.0f);
        std::shared_ptr<structures::IArray<std::shared_ptr<MapGridRect<std::string>>>> map_grid_rects =
                map_grid_dict->chebyshev_grid_rects_in_range(
                    lane_bounding_box.get_origin(),
                    lane_bounding_box.get_width() / 2.0f,
                    lane_bounding_box.get_height() / 2.0f);
        size_t i;
        for (i = 0; i < map_grid_rects->count(); ++i)
        {
            (*map_grid_rects)[i]->insert_lane(lane);
        }
    }

    rapidjson::Value::Object traffic_light_id_to_traffic_light_data = json_document["id_traffic_light_dict"].GetObject();

    for (rapidjson::Value::ConstMemberIterator traffic_light_id_to_traffic_light_itr = traffic_light_id_to_traffic_light_data.MemberBegin();
         traffic_light_id_to_traffic_light_itr != traffic_light_id_to_traffic_light_data.MemberEnd(); ++traffic_light_id_to_traffic_light_itr)
    {
        const std::string& traffic_light_id = traffic_light_id_to_traffic_light_itr->name.GetString();
        std::shared_ptr<LyftTrafficLight::IFaceDictionary> face_colour_to_face_type_dict;
        std::shared_ptr<LyftTrafficLight::TemporalStateDictionary> timestamp_to_state_dict;
        try
        {
            face_colour_to_face_type_dict = (*id_to_face_colour_to_face_type_dict)[traffic_light_id];
            timestamp_to_state_dict = (*id_to_timestamp_to_state_dict)[traffic_light_id];
        }
        catch (const std::exception&) {}
        std::shared_ptr<LyftTrafficLight> traffic_light(
            new LyftTrafficLight(traffic_light_id, shared_from_this(), face_colour_to_face_type_dict,
                                 timestamp_to_state_dict, traffic_light_id_to_traffic_light_itr->value.GetObject()));
        (*id_to_traffic_light_dict).update(traffic_light_id, traffic_light);
        map_grid_dict->chebyshev_proliferate(traffic_light->get_position(), 0.0f);
        (*map_grid_dict)[traffic_light->get_position()]->insert_traffic_light(traffic_light);
    }
}

std::shared_ptr<const ILane<std::string>> LyftMap::get_lane(std::string id) const
{
    try
    {
        return (*id_to_lane_dict)[id];
    }
    catch (const std::out_of_range&)
    {
        throw LyftMap::ObjectNotFound();
    }
}

std::shared_ptr<const ILaneArray<std::string>> LyftMap::get_encapsulating_lanes(geometry::Vec point) const
{
    if (map_grid_dict->contains(point))
    {
        std::shared_ptr<MapGridRect<std::string>> map_grid_rect = (*map_grid_dict)[point];
        if (map_grid_rect)
        {
            return map_grid_rect->get_encapsulating_lanes(point);
        }
    }

    return std::shared_ptr<const ILaneArray<std::string>>(new structures::stl::STLStackArray<std::shared_ptr<const ILane<std::string>>>());
}

std::shared_ptr<const ILaneArray<std::string>> LyftMap::get_lanes(std::shared_ptr<const structures::IArray<std::string>> ids) const
{
    const size_t lane_count = ids->count();
    std::shared_ptr<ILaneArray<std::string>> lanes(
                new structures::stl::STLStackArray<std::shared_ptr<const ILane<std::string>>>(lane_count));

    size_t i;
    for (i = 0; i < lane_count; ++i)
    {
        (*lanes)[i] = get_lane((*ids)[i]);
    }

    return lanes;
}

std::shared_ptr<const ILaneArray<std::string>> LyftMap::get_lanes_in_range(geometry::Vec point, FP_DATA_TYPE distance) const
{
    std::shared_ptr<structures::IArray<std::shared_ptr<MapGridRect<std::string>>>> map_grid_rects =
            map_grid_dict->chebyshev_grid_rects_in_range(point, distance);

    std::shared_ptr<structures::ISet<std::shared_ptr<const ILane<std::string>>>> lanes(
                new structures::stl::STLSet<std::shared_ptr<const ILane<std::string>>>());

    size_t i;
    for (i = 0; i < map_grid_rects->count(); ++i)
    {
        lanes->union_with(((*map_grid_rects)[i]->get_lanes()));
    }

    return lanes->get_array();
}

std::shared_ptr<const ITrafficLight<std::string>> LyftMap::get_traffic_light(std::string id) const
{
    try
    {
        return (*id_to_traffic_light_dict)[id];
    }
    catch (const std::out_of_range&)
    {
        throw LyftMap::ObjectNotFound();
    }
}

std::shared_ptr<const ITrafficLightArray<std::string>> LyftMap::get_traffic_lights(std::shared_ptr<const structures::IArray<std::string>> ids) const
{
    const size_t traffic_light_count = ids->count();
    std::shared_ptr<ITrafficLightArray<std::string>> traffic_lights(
                new structures::stl::STLStackArray<std::shared_ptr<const ITrafficLight<std::string>>>(traffic_light_count));

    size_t i;
    for (i = 0; i < traffic_light_count; ++i)
    {
        (*traffic_lights)[i] = get_traffic_light((*ids)[i]);
    }

    return traffic_lights;
}

std::shared_ptr<const ITrafficLightArray<std::string>> LyftMap::get_traffic_lights_in_range(geometry::Vec point, FP_DATA_TYPE distance) const
{
    std::shared_ptr<structures::IArray<std::shared_ptr<MapGridRect<std::string>>>> map_grid_rects =
            map_grid_dict->chebyshev_grid_rects_in_range(point, distance);

    std::shared_ptr<structures::ISet<std::shared_ptr<const ITrafficLight<std::string>>>> traffic_lights(
                new structures::stl::STLSet<std::shared_ptr<const ITrafficLight<std::string>>>());

    size_t i;
    for (i = 0; i < map_grid_rects->count(); ++i)
    {
        traffic_lights->union_with((*map_grid_rects)[i]->get_traffic_lights());
    }

    return traffic_lights->get_array();
}

std::shared_ptr<LyftMap> LyftMap::copy() const
{
    std::shared_ptr<LyftMap> map_copy(new LyftMap());
    map_copy->id_to_lane_dict.reset(
                new structures::stl::STLDictionary<std::string, std::shared_ptr<LyftLane>>(
                    *this->id_to_lane_dict));
    map_copy->id_to_traffic_light_dict.reset(
                new structures::stl::STLDictionary<std::string, std::shared_ptr<LyftTrafficLight>>(
                    *this->id_to_traffic_light_dict));
    map_copy->map_grid_dict.reset(new geometry::GridDictionary<MapGridRect<std::string>>(
                                      *this->map_grid_dict));
    return map_copy;
}

}
}
}
}
