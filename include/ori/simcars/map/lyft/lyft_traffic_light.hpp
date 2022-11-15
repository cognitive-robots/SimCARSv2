#pragma once

#include <ori/simcars/structures/dictionary_interface.hpp>
#include <ori/simcars/temporal/proximal_temporal_dictionary.hpp>
#include <ori/simcars/map/living_traffic_light_abstract.hpp>
#include <ori/simcars/map/lyft/lyft_declarations.hpp>
#include <ori/simcars/map/lyft/lyft_map.hpp>

#include <rapidjson/document.h>

namespace ori
{
namespace simcars
{
namespace map
{
namespace lyft
{

class LyftTrafficLight : public ALivingTrafficLight<std::string>
{
    geometry::Vec position;
    FP_DATA_TYPE orientation;
    ITrafficLightStateHolder::IFaceDictionary* const face_colour_to_face_type_dict;
    ITrafficLightStateHolder::TemporalStateDictionary* const timestamp_to_state_dict;

public:
    LyftTrafficLight(std::string const &id, IMap<std::string> const *map,
                     ITrafficLightStateHolder::IFaceDictionary *face_colour_to_face_type_dict,
                     ITrafficLightStateHolder::TemporalStateDictionary *timestamp_to_state_dict,
                     rapidjson::Value::ConstObject const &json_traffic_light_data);
    ~LyftTrafficLight();

    ITrafficLightStateHolder::State const* get_state(temporal::Time timestamp) const override;

    geometry::Vec const& get_position() const override;
    FP_DATA_TYPE get_orientation() const override;
    structures::IArray<ITrafficLightStateHolder::FaceColour> const* get_face_colours() const override;
    ITrafficLightStateHolder::FaceType get_face_type(ITrafficLightStateHolder::FaceColour face_colour) const override;
};

}
}
}
}
