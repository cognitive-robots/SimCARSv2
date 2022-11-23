
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/map/lyft/lyft_map.hpp>

#include <lz4_stream.h>
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

void LyftMap::save_virt(std::ofstream &output_filestream) const
{
    throw utils::NotImplementedException();
}

void LyftMap::load_virt(std::ifstream &input_filestream)
{
    lz4_stream::istream input_lz4_stream(input_filestream);
    rapidjson::BasicIStreamWrapper input_lz4_json_stream(input_lz4_stream);

    rapidjson::Document json_document;
    json_document.ParseStream(input_lz4_json_stream);

    id_to_lane_dict = new structures::stl::STLDictionary<std::string, LyftLane*>();
    id_to_traffic_light_dict = new structures::stl::STLDictionary<std::string, LyftTrafficLight*>();

    stray_ghosts = new structures::stl::STLSet<IMapObject<std::string> const*>();

    map_grid_dict = new geometry::GridDictionary<MapGridRect<std::string>>(geometry::Vec(0, 0), 100);

    structures::IDictionary<std::string, ITrafficLightStateHolder::IFaceDictionary*> *id_to_face_colour_to_face_type_dict =
                new structures::stl::STLDictionary<std::string, ITrafficLightStateHolder::IFaceDictionary*>;
    structures::IDictionary<std::string, ITrafficLightStateHolder::TemporalStateDictionary*> *id_to_timestamp_to_state_dict =
                new structures::stl::STLDictionary<std::string, ITrafficLightStateHolder::TemporalStateDictionary*>;

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
        std::string const &traffic_light_face_id =
                traffic_light_face_id_to_traffic_light_face_itr->name.GetString();
        if (!traffic_light_face_id_to_traffic_light_id.HasMember(traffic_light_face_id.c_str()))
        {
            continue;
        }
        std::string const &traffic_light_id =
                traffic_light_face_id_to_traffic_light_id[traffic_light_face_id.c_str()].GetString();

        ITrafficLightStateHolder::IFaceDictionary *face_colour_to_face_type_dict;
        if (id_to_face_colour_to_face_type_dict->contains(traffic_light_id))
        {
            face_colour_to_face_type_dict = (*id_to_face_colour_to_face_type_dict)[traffic_light_id];
        }
        else
        {
            face_colour_to_face_type_dict =
                    new structures::stl::STLDictionary<ITrafficLightStateHolder::FaceColour, ITrafficLightStateHolder::FaceType>();
            id_to_face_colour_to_face_type_dict->update(traffic_light_id, face_colour_to_face_type_dict);
        }

        rapidjson::Value::ConstObject const &traffic_light_face_data =
                traffic_light_face_id_to_traffic_light_face_itr->value.GetObject();
        std::string const &traffic_light_face_type_str = traffic_light_face_data["traffic_light_face_type"].GetString();

        ITrafficLightStateHolder::FaceColour face_colour = ITrafficLightStateHolder::FaceColour::UNKNOWN;
        ITrafficLightStateHolder::FaceType face_type = ITrafficLightStateHolder::FaceType::UNKNOWN;

        std::string token;
        size_t current_delim, next_delim;
        for (current_delim = 0, next_delim = traffic_light_face_type_str.find('_');
             current_delim == next_delim;
             current_delim = next_delim, next_delim = traffic_light_face_type_str.find('_', current_delim))
        {
            token = traffic_light_face_type_str.substr(current_delim, next_delim - current_delim);

            if (token == "signal")
            {
                face_type = ITrafficLightStateHolder::FaceType::STANDARD;
            }
            else if (token == "red")
            {
                face_colour = ITrafficLightStateHolder::FaceColour::RED;
            }
            else if (token == "yellow")
            {
                face_colour = ITrafficLightStateHolder::FaceColour::YELLOW;
            }
            else if (token == "green")
            {
                face_colour = ITrafficLightStateHolder::FaceColour::GREEN;
            }
            else if (token == "arrow")
            {
                face_type = ITrafficLightStateHolder::FaceType(static_cast<int>(face_type) | static_cast<int>(ITrafficLightStateHolder::FaceType::ARROW));
            }
            else if (token == "left")
            {
                face_type = ITrafficLightStateHolder::FaceType(static_cast<int>(face_type) | static_cast<int>(ITrafficLightStateHolder::FaceType::LEFT));
            }
            else if (token == "right")
            {
                face_type = ITrafficLightStateHolder::FaceType(static_cast<int>(face_type) | static_cast<int>(ITrafficLightStateHolder::FaceType::RIGHT));
            }
            else if (token == "upper")
            {
                face_type = ITrafficLightStateHolder::FaceType(static_cast<int>(face_type) | static_cast<int>(ITrafficLightStateHolder::FaceType::UPPER));
            }
            else if (token == "u")
            {
                current_delim = next_delim;
                next_delim = traffic_light_face_type_str.find('_', current_delim);
                token = traffic_light_face_type_str.substr(current_delim, next_delim - current_delim);

                if (token == "turn")
                {
                    face_type = ITrafficLightStateHolder::FaceType(static_cast<int>(face_type) | static_cast<int>(ITrafficLightStateHolder::FaceType::UTURN));
                }
                else
                {
                    face_type = ITrafficLightStateHolder::FaceType::UNKNOWN;
                }
            }
            else if (token == "flashing")
            {
                face_type = ITrafficLightStateHolder::FaceType(static_cast<int>(face_type) | static_cast<int>(ITrafficLightStateHolder::FaceType::FLASHING));
            }
            else
            {
                face_type = ITrafficLightStateHolder::FaceType::UNKNOWN;
            }
        }

        face_colour_to_face_type_dict->update(face_colour, face_type);


        ITrafficLightStateHolder::TemporalStateDictionary *timestamp_to_state_dict;
        if (id_to_timestamp_to_state_dict->contains(traffic_light_id))
        {
            timestamp_to_state_dict = (*id_to_timestamp_to_state_dict)[traffic_light_id];
        }
        else
        {
            timestamp_to_state_dict =
                    new ITrafficLightStateHolder::TemporalStateDictionary(temporal::Duration(50), 10);
            id_to_timestamp_to_state_dict->update(traffic_light_id, timestamp_to_state_dict);
        }

        if (traffic_light_face_id_to_state_data.HasMember(traffic_light_face_id.c_str()))
        {
            rapidjson::Value::Array state_data = traffic_light_face_id_to_state_data[traffic_light_face_id.c_str()].GetArray();
            for (rapidjson::Value const &state_data_element : state_data)
            {
                rapidjson::Value::ConstArray const &state_pair = state_data_element.GetArray();
                temporal::Time timestamp(temporal::Duration(state_pair[0].GetInt64()));
                LyftTrafficLightFaceState face_state = static_cast<LyftTrafficLightFaceState>(state_pair[1].GetInt());

                if (face_state == LyftTrafficLightFaceState::ACTIVE)
                {
                    if (timestamp_to_state_dict->contains(timestamp))
                    {
                        // TODO: Find a better way of handling duplicate timestamps, likely the result of multiple lights
                        // being active at once on a traffic light
                        delete (*timestamp_to_state_dict)[timestamp];
                    }
                    timestamp_to_state_dict->update(
                                timestamp,
                                new ITrafficLightStateHolder::State(face_colour));
                }
            }
        }
    }

    rapidjson::Value::Object lane_id_to_lane_data = json_document["id_lane_dict"].GetObject();

    for (rapidjson::Value::ConstMemberIterator lane_id_to_lane_itr = lane_id_to_lane_data.MemberBegin();
         lane_id_to_lane_itr != lane_id_to_lane_data.MemberEnd(); ++lane_id_to_lane_itr)
    {
        std::string const &lane_id = lane_id_to_lane_itr->name.GetString();
        LyftLane *lane = new LyftLane(lane_id, this, lane_id_to_lane_itr->value.GetObject());
        id_to_lane_dict->update(lane_id, lane);

        geometry::Rect const &lane_bounding_box = lane->get_bounding_box();
        map_grid_dict->chebyshev_proliferate(
            lane_bounding_box.get_origin(),
            lane_bounding_box.get_width() / 2.0f,
            lane_bounding_box.get_height() / 2.0f);
        structures::IArray<MapGridRect<std::string>*> *map_grid_rects =
                map_grid_dict->chebyshev_grid_rects_in_range(
                    lane_bounding_box.get_origin(),
                    lane_bounding_box.get_width() / 2.0f,
                    lane_bounding_box.get_height() / 2.0f);
        for (size_t i = 0; i < map_grid_rects->count(); ++i)
        {
            (*map_grid_rects)[i]->insert_lane(lane);
        }
        delete map_grid_rects;
    }

    rapidjson::Value::Object traffic_light_id_to_traffic_light_data = json_document["id_traffic_light_dict"].GetObject();

    for (rapidjson::Value::ConstMemberIterator traffic_light_id_to_traffic_light_itr = traffic_light_id_to_traffic_light_data.MemberBegin();
         traffic_light_id_to_traffic_light_itr != traffic_light_id_to_traffic_light_data.MemberEnd(); ++traffic_light_id_to_traffic_light_itr)
    {
        std::string const &traffic_light_id = traffic_light_id_to_traffic_light_itr->name.GetString();

        ITrafficLightStateHolder::IFaceDictionary *face_colour_to_face_type_dict = nullptr;
        ITrafficLightStateHolder::TemporalStateDictionary *timestamp_to_state_dict = nullptr;
        try
        {
            face_colour_to_face_type_dict = (*id_to_face_colour_to_face_type_dict)[traffic_light_id];
            timestamp_to_state_dict = (*id_to_timestamp_to_state_dict)[traffic_light_id];
        }
        catch (std::exception const&) {}

        LyftTrafficLight *traffic_light =
            new LyftTrafficLight(traffic_light_id, this, face_colour_to_face_type_dict,
                                 timestamp_to_state_dict, traffic_light_id_to_traffic_light_itr->value.GetObject());
        id_to_traffic_light_dict->update(traffic_light_id, traffic_light);
        map_grid_dict->chebyshev_proliferate(traffic_light->get_position(), 0.0f);
        (*map_grid_dict)[traffic_light->get_position()]->insert_traffic_light(traffic_light);

        if (face_colour_to_face_type_dict != nullptr)
        {
            id_to_face_colour_to_face_type_dict->erase(traffic_light_id);
        }
        if (timestamp_to_state_dict != nullptr)
        {
            id_to_timestamp_to_state_dict->erase(traffic_light_id);
        }
    }

    structures::IArray<ITrafficLightStateHolder::IFaceDictionary*> const *face_colour_to_face_type_dict_array =
            id_to_face_colour_to_face_type_dict->get_values();
    for (size_t i = 0; i < face_colour_to_face_type_dict_array->count(); ++i)
    {
        delete (*face_colour_to_face_type_dict_array)[i];
    }
    delete id_to_face_colour_to_face_type_dict;

    structures::IArray<ITrafficLightStateHolder::TemporalStateDictionary*> const *timestamp_to_state_dict_array =
            id_to_timestamp_to_state_dict->get_values();
    for (size_t i = 0; i < timestamp_to_state_dict_array->count(); ++i)
    {
        structures::IArray<ITrafficLightStateHolder::State const*> const *state_array =
                (*timestamp_to_state_dict_array)[i]->get_values();
        for (size_t j = 0; j < state_array->count(); ++j)
        {
            delete (*state_array)[j];
        }
        delete (*timestamp_to_state_dict_array)[i];
    }
    delete id_to_timestamp_to_state_dict;
}

LyftMap::~LyftMap()
{
    size_t i;

    structures::IArray<LyftLane*> const *lane_array =
            id_to_lane_dict->get_values();
    for (i = 0; i < lane_array->count(); ++i)
    {
        delete (*lane_array)[i];
    }
    delete id_to_lane_dict;

    structures::IArray<LyftTrafficLight*> const *traffic_light_array =
            id_to_traffic_light_dict->get_values();
    for (i = 0; i < traffic_light_array->count(); ++i)
    {
        delete (*traffic_light_array)[i];
    }
    delete id_to_traffic_light_dict;

    structures::IArray<IMapObject<std::string> const*> const *ghost_array =
            stray_ghosts->get_array();
    for (i = 0; i < ghost_array->count(); ++i)
    {
        delete (*ghost_array)[i];
    }
    delete stray_ghosts;

    delete map_grid_dict;
}

ILane<std::string> const* LyftMap::get_lane(std::string id) const
{
    try
    {
        return (*id_to_lane_dict)[id];
    }
    catch (std::out_of_range const&)
    {
        throw IMap::ObjectNotFound();
    }
}

ILaneArray<std::string> const* LyftMap::get_encapsulating_lanes(geometry::Vec point) const
{
    if (map_grid_dict->contains(point))
    {
        MapGridRect<std::string> *map_grid_rect = (*map_grid_dict)[point];
        if (map_grid_rect)
        {
            return map_grid_rect->get_encapsulating_lanes(point);
        }
    }

    return new LivingLaneStackArray<std::string>;
}

ILaneArray<std::string> const* LyftMap::get_lanes(structures::IArray<std::string> const *ids) const
{
    const size_t lane_count = ids->count();
    ILaneArray<std::string> *lanes =
            new LivingLaneStackArray<std::string>(lane_count);

    for (size_t i = 0; i < lane_count; ++i)
    {
        (*lanes)[i] = get_lane((*ids)[i]);
    }

    return lanes;
}

ILaneArray<std::string> const* LyftMap::get_lanes_in_range(geometry::Vec point, FP_DATA_TYPE distance) const
{
    structures::IArray<MapGridRect<std::string>*> *map_grid_rects =
            map_grid_dict->chebyshev_grid_rects_in_range(point, distance);

    structures::stl::STLSet<ILane<std::string> const*> lanes;

    for (size_t i = 0; i < map_grid_rects->count(); ++i)
    {
        lanes.union_with((*map_grid_rects)[i]->get_lanes());
    }

    LivingLaneStackArray<std::string> *lane_array = new LivingLaneStackArray<std::string>;

    lanes.get_array(lane_array);

    delete map_grid_rects;

    return lane_array;
}

ITrafficLight<std::string> const* LyftMap::get_traffic_light(std::string id) const
{
    try
    {
        return (*id_to_traffic_light_dict)[id];
    }
    catch (std::out_of_range const&)
    {
        throw IMap::ObjectNotFound();
    }
}

ITrafficLightArray<std::string> const* LyftMap::get_traffic_lights(structures::IArray<std::string> const *ids) const
{
    size_t const traffic_light_count = ids->count();
    ITrafficLightArray<std::string> *traffic_lights =
            new LivingTrafficLightStackArray<std::string>(traffic_light_count);

    for (size_t i = 0; i < traffic_light_count; ++i)
    {
        (*traffic_lights)[i] = get_traffic_light((*ids)[i]);
    }

    return traffic_lights;
}

ITrafficLightArray<std::string> const* LyftMap::get_traffic_lights_in_range(geometry::Vec point, FP_DATA_TYPE distance) const
{
    structures::IArray<MapGridRect<std::string>*> *map_grid_rects =
            map_grid_dict->chebyshev_grid_rects_in_range(point, distance);

    structures::stl::STLSet<ITrafficLight<std::string> const*> traffic_lights;

    for (size_t i = 0; i < map_grid_rects->count(); ++i)
    {
        traffic_lights.union_with((*map_grid_rects)[i]->get_traffic_lights());
    }

    LivingTrafficLightStackArray<std::string> *traffic_light_array = new LivingTrafficLightStackArray<std::string>;

    traffic_lights.get_array(traffic_light_array);

    delete map_grid_rects;

    return traffic_light_array;
}

void LyftMap::register_stray_ghost(IMapObject<std::string> const *ghost) const
{
    stray_ghosts->insert(ghost);
}

void LyftMap::unregister_stray_ghost(IMapObject<std::string> const *ghost) const
{
    stray_ghosts->erase(ghost);
}

LyftMap* LyftMap::shallow_copy() const
{
    LyftMap *map_copy = new LyftMap;

    map_copy->id_to_lane_dict =
                new structures::stl::STLDictionary<std::string, LyftLane*>(
                    this->id_to_lane_dict);
    map_copy->id_to_traffic_light_dict =
                new structures::stl::STLDictionary<std::string, LyftTrafficLight*>(
                    this->id_to_traffic_light_dict);

    map_copy->stray_ghosts = new structures::stl::STLSet<IMapObject<std::string> const*>(this->stray_ghosts);

    map_copy->map_grid_dict = new geometry::GridDictionary<MapGridRect<std::string>>(
                                      *this->map_grid_dict);
    return map_copy;
}

}
}
}
}
