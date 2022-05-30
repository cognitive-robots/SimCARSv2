#pragma once

#include <ori/simcars/temporal/temporal_dictionary.hpp>
#include <ori/simcars/agent/agent_abstract.hpp>
#include <ori/simcars/agent/lyft/lyft_declarations.hpp>

#include <rapidjson/document.h>

#include <string>

namespace ori
{
namespace simcars
{
namespace agent
{
namespace lyft
{

class LyftAgent : virtual public AAgent
{
    temporal::TemporalDictionary<std::shared_ptr<LyftAgent::State>> timestamp_to_state_dict;
    uint32_t id;
    bool ego;
    FP_DATA_TYPE bounding_box_length, bounding_box_width;
    LyftAgent::Class class_value;
    const std::weak_ptr<const IScene> scene;

public:
    LyftAgent(std::shared_ptr<const IScene> scene, const rapidjson::Value::ConstObject& json_agent_data);

    std::shared_ptr<const LyftAgent::State> get_state(temporal::Time timestamp) const override;

    temporal::Time get_birth() const override;
    temporal::Time get_death() const override;

    uint32_t get_id() const override;
    bool is_ego() const override;
    bool is_ever_simulated() const override;
    bool is_simulated(temporal::Time timestamp) const override;
    FP_DATA_TYPE get_length() const override;
    FP_DATA_TYPE get_width() const override;
    LyftAgent::Class get_class() const override;
    std::shared_ptr<const IScene> get_scene() const override;
};

}
}
}
}
