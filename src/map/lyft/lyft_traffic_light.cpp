
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

LyftTrafficLight::LyftTrafficLight(const std::string& id, std::shared_ptr<const IMap<std::string>> map,
                                   std::shared_ptr<IFaceDictionary> face_colour_to_face_type_dict,
                                   std::shared_ptr<TemporalStateDictionary> timestamp_to_state_dict,
                                   const rapidjson::Value::ConstObject& json_traffic_light_data)
    : ALivingTrafficLight(id, map), face_colour_to_face_type_dict(face_colour_to_face_type_dict),
      timestamp_to_state_dict(timestamp_to_state_dict)
{
    const rapidjson::Value::ConstArray position_data = json_traffic_light_data["coord"].GetArray();

    position(0) = position_data[0].GetDouble();
    position(1) = position_data[1].GetDouble();

    orientation = M_PI * json_traffic_light_data["bearing_degrees"].GetDouble() / 180.0;
}

std::shared_ptr<const LyftTrafficLight::State> LyftTrafficLight::get_state(temporal::Time timestamp) const
{
    if (timestamp_to_state_dict && timestamp_to_state_dict->contains(timestamp))
    {
        return (*timestamp_to_state_dict)[timestamp];
    }
    else
    {
        return std::shared_ptr<const LyftTrafficLight::State>(new LyftTrafficLight::State());
    }
}

const geometry::Vec& LyftTrafficLight::get_position() const
{
    return position;
}

FP_DATA_TYPE LyftTrafficLight::get_orientation() const
{
    return orientation;
}

std::shared_ptr<const structures::IArray<LyftTrafficLight::FaceColour>> LyftTrafficLight::get_face_colours() const
{
    if (!face_colour_to_face_type_dict)
    {
        throw std::runtime_error("Traffic light face colours not available");
    }

    return face_colour_to_face_type_dict->get_keys();
}

LyftTrafficLight::FaceType LyftTrafficLight::get_face_type(LyftTrafficLight::FaceColour face_colour) const
{
    if (face_colour_to_face_type_dict && face_colour_to_face_type_dict->contains(face_colour))
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
