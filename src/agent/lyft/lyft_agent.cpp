
#include <ori/simcars/agent/inattentive_simulated_agent.hpp>
#include <ori/simcars/agent/lyft/lyft_agent.hpp>

#include <neargye/magic_enum/magic_enum.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace agent
{
namespace lyft
{

LyftAgent::LyftAgent(std::shared_ptr<const IScene> scene, const rapidjson::Value::ConstObject& json_agent_data)
    : timestamp_to_state_dict(temporal::Duration(75), 10), scene(scene)
{
    id = json_agent_data["id"].GetInt();
    ego = json_agent_data["ego"].GetBool();
    const rapidjson::Value::ConstArray bounding_box_data = json_agent_data["bounding_box"].GetArray();
    bounding_box_length = bounding_box_data[0].GetDouble();
    bounding_box_width = bounding_box_data[1].GetDouble();

    std::string class_label = json_agent_data["class_label"].GetString();
    size_t first_underscore = class_label.find('_');
    size_t second_underscore = class_label.find('_', first_underscore + 1);
    class_label = class_label.substr(second_underscore + 1);
    auto temp_class_value = magic_enum::enum_cast<LyftAgent::Class>(class_label);
    if (temp_class_value.has_value())
    {
        class_value = temp_class_value.value();
    }
    else
    {
        class_value = LyftAgent::Class::UNKNOWN;
    }

    const rapidjson::Value::ConstArray state_data = json_agent_data["states"].GetArray();
    const size_t state_data_size = state_data.Capacity();

    size_t i;
    for (i = 0; i < state_data_size; ++i)
    {
        const rapidjson::Value::ConstObject state_data_entry = state_data[i].GetObject();
        const temporal::Time timestamp(temporal::Duration(state_data_entry["timestamp"].GetInt64()));

        const rapidjson::Value::ConstArray position_data = state_data_entry["position"].GetArray();
        const geometry::Vec position(position_data[0].GetDouble(), position_data[1].GetDouble());
        const rapidjson::Value::ConstArray linear_velocity_data = state_data_entry["linear_velocity"].GetArray();
        const geometry::Vec linear_velocity(linear_velocity_data[0].GetDouble(), linear_velocity_data[1].GetDouble());
        const rapidjson::Value::ConstArray linear_acceleration_data = state_data_entry["linear_acceleration"].GetArray();
        const geometry::Vec linear_acceleration(linear_acceleration_data[0].GetDouble(), linear_acceleration_data[1].GetDouble());

        const FP_DATA_TYPE rotation(state_data_entry["rotation"].GetDouble());
        const FP_DATA_TYPE angular_velocity(state_data_entry["angular_velocity"].GetDouble());
        const FP_DATA_TYPE angular_acceleration(state_data_entry["angular_acceleration"].GetDouble());

        // NOTE: Assumption made that all ground truth data was captured under nominal conditions i.e. No crashes, etc.
        const std::shared_ptr<LyftAgent::State> state(new LyftAgent::State(position, rotation, linear_velocity, angular_velocity,
                                     linear_acceleration, angular_acceleration, LyftAgent::Status::NOMINAL));
        timestamp_to_state_dict.update(timestamp, state);
    }
}

std::shared_ptr<const LyftAgent::State> LyftAgent::get_state(temporal::Time timestamp) const
{
    return timestamp_to_state_dict[timestamp];
}

temporal::Time LyftAgent::get_birth() const
{
    return timestamp_to_state_dict.get_earliest_timestamp();
}

temporal::Time LyftAgent::get_death() const
{
    return timestamp_to_state_dict.get_latest_timestamp();
}

uint32_t LyftAgent::get_id() const
{
    return id;
}

bool LyftAgent::is_ego() const
{
    return ego;
}

bool LyftAgent::is_ever_simulated() const
{
    return false;
}

bool LyftAgent::is_simulated(temporal::Time timestamp) const
{
    return false;
}

FP_DATA_TYPE LyftAgent::get_length() const
{
    return bounding_box_length;
}

FP_DATA_TYPE LyftAgent::get_width() const
{
    return bounding_box_width;
}

LyftAgent::Class LyftAgent::get_class() const
{
    return class_value;
}

std::shared_ptr<const IScene> LyftAgent::get_scene() const
{
    return scene.lock();
}

}
}
}
}
