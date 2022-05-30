#pragma once

#include <ori/simcars/structures/dictionary_interface.hpp>
#include <ori/simcars/temporal/proximal_temporal_dictionary.hpp>
#include <ori/simcars/map/living_traffic_light_abstract.hpp>
#include <ori/simcars/map/lyft/lyft_declarations.hpp>
#include <ori/simcars/map/lyft/lyft_map.hpp>

#include <rapidjson/document.h>

#include <map>
#include <vector>
#include <tuple>

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
    std::shared_ptr<structures::IDictionary<LyftTrafficLight::FaceColour, LyftTrafficLight::FaceType>> const face_colour_to_face_type_dict;
    std::shared_ptr<temporal::ProximalTemporalDictionary<std::shared_ptr<const LyftTrafficLight::State>>> const timestamp_to_state_dict;

public:
    typedef structures::IDictionary<ATrafficLight::FaceColour, ATrafficLight::FaceType> IFaceDictionary;
    typedef temporal::ProximalTemporalDictionary<std::shared_ptr<const LyftTrafficLight::State>> TemporalStateDictionary;

    LyftTrafficLight(const std::string& id, std::shared_ptr<const IMap<std::string>> map,
                     std::shared_ptr<IFaceDictionary> face_colour_to_face_type_dict,
                     std::shared_ptr<TemporalStateDictionary> timestamp_to_state_dict,
                     const rapidjson::Value::ConstObject& json_traffic_light_data);

    std::shared_ptr<const LyftTrafficLight::State> get_state(temporal::Time timestamp) const override;

    const geometry::Vec& get_position() const override;
    FP_DATA_TYPE get_orientation() const override;
    std::shared_ptr<const structures::IArray<LyftTrafficLight::FaceColour>> get_face_colours() const override;
    LyftTrafficLight::FaceType get_face_type(LyftTrafficLight::FaceColour face_colour) const override;
};

}
}
}
}
