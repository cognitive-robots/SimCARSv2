
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/map/lyft/lyft_traffic_light.hpp>

#include <cassert>

namespace ori
{
namespace simcars
{
namespace map
{
namespace lyft
{

LyftTrafficLight::LyftTrafficLight(std::string const &id, IMap<std::string> const *map,
                                   ITrafficLightStateHolder::IFaceDictionary *face_colour_to_face_type_dict,
                                   ITrafficLightStateHolder::TemporalStateDictionary *timestamp_to_state_dict,
                                   rapidjson::Value::ConstObject const &json_traffic_light_data)
    : ALivingTrafficLight(id, map), face_colour_to_face_type_dict(face_colour_to_face_type_dict),
      timestamp_to_state_dict(timestamp_to_state_dict)
{
    rapidjson::Value::ConstArray const &position_data = json_traffic_light_data["coord"].GetArray();

    position(0) = position_data[0].GetDouble();
    position(1) = position_data[1].GetDouble();

    orientation = M_PI * json_traffic_light_data["bearing_degrees"].GetDouble() / 180.0;
}

LyftTrafficLight::~LyftTrafficLight()
{
    delete face_colour_to_face_type_dict;

    if (timestamp_to_state_dict != nullptr)
    {
        structures::IArray<ITrafficLightStateHolder::State const*> const *state_array = timestamp_to_state_dict->get_values();
        for (size_t i = 0; i < state_array->count(); ++i)
        {
            delete (*state_array)[i];
        }
    }
    delete timestamp_to_state_dict;
}

ITrafficLightStateHolder::State const* LyftTrafficLight::get_state(temporal::Time timestamp) const
{
    if (timestamp_to_state_dict != nullptr && timestamp_to_state_dict->contains(timestamp))
    {
        return (*timestamp_to_state_dict)[timestamp];
    }
    else
    {
        return nullptr;
    }
}

geometry::Vec const& LyftTrafficLight::get_position() const
{
    return position;
}

FP_DATA_TYPE LyftTrafficLight::get_orientation() const
{
    return orientation;
}

structures::IArray<ITrafficLightStateHolder::FaceColour> const* LyftTrafficLight::get_face_colours() const
{
    if (face_colour_to_face_type_dict == nullptr)
    {
        throw std::runtime_error("Traffic light face colours not available");
    }

    return face_colour_to_face_type_dict->get_keys();
}

ITrafficLightStateHolder::FaceType LyftTrafficLight::get_face_type(ITrafficLightStateHolder::FaceColour face_colour) const
{
    if (face_colour_to_face_type_dict != nullptr && face_colour_to_face_type_dict->contains(face_colour))
    {
        return (*face_colour_to_face_type_dict)[face_colour];
    }
    else
    {
        return ATrafficLight::FaceType::UNKNOWN;
    }
}

}
}
}
}
