
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_concat_array.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/agent/constant.hpp>
#include <ori/simcars/agent/event.hpp>
#include <ori/simcars/agent/variable.hpp>
#include <ori/simcars/agent/lyft/lyft_driving_agent.hpp>

#include <neargye/magic_enum/magic_enum.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{
namespace lyft
{

LyftDrivingAgent::LyftDrivingAgent() {}

LyftDrivingAgent::LyftDrivingAgent(const rapidjson::Value::ConstObject& json_agent_data)
{
    this->min_temporal_limit = temporal::Time::max();
    this->max_temporal_limit = temporal::Time::min();

    FP_DATA_TYPE min_position_x = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_position_x = std::numeric_limits<FP_DATA_TYPE>::min();
    FP_DATA_TYPE min_position_y = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_position_y = std::numeric_limits<FP_DATA_TYPE>::min();

    const uint32_t id = json_agent_data["id"].GetInt();
    const bool ego = json_agent_data["ego"].GetBool();

    this->name = (ego ? "ego_vehicle_" : "non_ego_vehicle_") + std::to_string(id);

    const std::shared_ptr<IConstant<uint32_t>> id_constant(new Constant<uint32_t>(this->name, "id", id));
    this->constant_dict.update(id_constant->get_full_name(), id_constant);

    const std::shared_ptr<IConstant<bool>> ego_constant(new Constant<bool>(this->name, "ego", ego));
    this->constant_dict.update(ego_constant->get_full_name(), ego_constant);

    const rapidjson::Value::ConstArray bounding_box_data = json_agent_data["bounding_box"].GetArray();
    const FP_DATA_TYPE bb_length = bounding_box_data[0].GetDouble();
    const FP_DATA_TYPE bb_width = bounding_box_data[1].GetDouble();

    const std::shared_ptr<IConstant<FP_DATA_TYPE>> bb_length_constant(new Constant<FP_DATA_TYPE>(this->name, "bb_length", bb_length));
    this->constant_dict.update(bb_length_constant->get_full_name(), bb_length_constant);

    const std::shared_ptr<IConstant<FP_DATA_TYPE>> bb_width_constant(new Constant<FP_DATA_TYPE>(this->name, "bb_width", bb_width));
    this->constant_dict.update(bb_width_constant->get_full_name(), bb_width_constant);

    std::string class_label = json_agent_data["class_label"].GetString();
    const size_t first_underscore = class_label.find('_');
    const size_t second_underscore = class_label.find('_', first_underscore + 1);
    class_label = class_label.substr(second_underscore + 1);
    auto temp_class_value = magic_enum::enum_cast<DrivingAgentClass>(class_label);
    DrivingAgentClass class_value;
    if (temp_class_value.has_value())
    {
        class_value = temp_class_value.value();
    }
    else
    {
        class_value = DrivingAgentClass::UNKNOWN;
    }

    const std::shared_ptr<IConstant<DrivingAgentClass>> road_agent_class_constant(new Constant<DrivingAgentClass>(this->name, "road_agent_class", class_value));
    this->constant_dict.update(road_agent_class_constant->get_full_name(), road_agent_class_constant);

    const rapidjson::Value::ConstArray state_data = json_agent_data["states"].GetArray();
    const size_t state_data_size = state_data.Capacity();

    const std::shared_ptr<IVariable<geometry::Vec>> position_variable(new Variable<geometry::Vec>(this->name, "position", IValuelessVariable::Type::BASE));
    this->variable_dict.update(position_variable->get_full_name(), position_variable);

    const std::shared_ptr<IVariable<geometry::Vec>> linear_velocity_variable(new Variable<geometry::Vec>(this->name, "linear_velocity", IValuelessVariable::Type::BASE));
    this->variable_dict.update(linear_velocity_variable->get_full_name(), linear_velocity_variable);

    const std::shared_ptr<IVariable<FP_DATA_TYPE>> aligned_linear_velocity_variable(new Variable<FP_DATA_TYPE>(this->name, "aligned_linear_velocity", IValuelessVariable::Type::BASE));
    this->variable_dict.update(aligned_linear_velocity_variable->get_full_name(), aligned_linear_velocity_variable);

    const std::shared_ptr<IVariable<geometry::Vec>> linear_acceleration_variable(new Variable<geometry::Vec>(this->name, "linear_acceleration", IValuelessVariable::Type::BASE));
    this->variable_dict.update(linear_acceleration_variable->get_full_name(), linear_acceleration_variable);

    const std::shared_ptr<IVariable<FP_DATA_TYPE>> aligned_linear_acceleration_variable(new Variable<FP_DATA_TYPE>(this->name, "aligned_linear_acceleration", IValuelessVariable::Type::INDIRECT_ACTUATION));
    this->variable_dict.update(aligned_linear_acceleration_variable->get_full_name(), aligned_linear_acceleration_variable);

    const std::shared_ptr<IVariable<FP_DATA_TYPE>> rotation_variable(new Variable<FP_DATA_TYPE>(this->name, "rotation", IValuelessVariable::Type::BASE));
    this->variable_dict.update(rotation_variable->get_full_name(), rotation_variable);

    const std::shared_ptr<IVariable<FP_DATA_TYPE>> steer_variable(new Variable<FP_DATA_TYPE>(this->name, "steer", IValuelessVariable::Type::INDIRECT_ACTUATION));
    this->variable_dict.update(steer_variable->get_full_name(), steer_variable);

    std::shared_ptr<const geometry::TrigBuff> trig_buff = geometry::TrigBuff::get_instance();

    size_t i;
    for (i = 0; i < state_data_size; ++i)
    {
        const rapidjson::Value::ConstObject state_data_entry = state_data[i].GetObject();
        const temporal::Time timestamp(temporal::Duration(state_data_entry["timestamp"].GetInt64()));
        this->min_temporal_limit = std::min(timestamp, this->min_temporal_limit);
        this->max_temporal_limit = std::max(timestamp, this->max_temporal_limit);

        const rapidjson::Value::ConstArray position_data = state_data_entry["position"].GetArray();
        const FP_DATA_TYPE position_x = position_data[0].GetDouble();
        const FP_DATA_TYPE position_y = position_data[1].GetDouble();
        min_position_x = std::min(position_x, min_position_x);
        max_position_x = std::max(position_x, max_position_x);
        min_position_y = std::min(position_y, min_position_y);
        max_position_y = std::max(position_y, max_position_y);
        const geometry::Vec position(position_x, position_y);
        const std::shared_ptr<IEvent<geometry::Vec>> position_event(new Event<geometry::Vec>(position_variable->get_full_name(), position, timestamp));
        position_variable->add_event(position_event);

        const rapidjson::Value::ConstArray linear_velocity_data = state_data_entry["linear_velocity"].GetArray();
        const geometry::Vec linear_velocity(linear_velocity_data[0].GetDouble(), linear_velocity_data[1].GetDouble());
        const std::shared_ptr<IEvent<geometry::Vec>> linear_velocity_event(new Event<geometry::Vec>(linear_velocity_variable->get_full_name(), linear_velocity, timestamp));
        linear_velocity_variable->add_event(linear_velocity_event);

        const rapidjson::Value::ConstArray linear_acceleration_data = state_data_entry["linear_acceleration"].GetArray();
        const geometry::Vec linear_acceleration(linear_acceleration_data[0].GetDouble(), linear_acceleration_data[1].GetDouble());
        const std::shared_ptr<IEvent<geometry::Vec>> linear_acceleration_event(new Event<geometry::Vec>(linear_acceleration_variable->get_full_name(), linear_acceleration, timestamp));
        linear_acceleration_variable->add_event(linear_acceleration_event);

        const FP_DATA_TYPE rotation(state_data_entry["rotation"].GetDouble());
        const std::shared_ptr<IEvent<FP_DATA_TYPE>> rotation_event(new Event<FP_DATA_TYPE>(rotation_variable->get_full_name(), rotation, timestamp));
        rotation_variable->add_event(rotation_event);

        FP_DATA_TYPE aligned_linear_velocity = (trig_buff->get_rot_mat(-rotation) * linear_velocity).x();
        std::shared_ptr<IEvent<FP_DATA_TYPE>> aligned_linear_velocity_event(new Event<FP_DATA_TYPE>(aligned_linear_velocity_variable->get_full_name(), aligned_linear_velocity, timestamp));
        aligned_linear_velocity_variable->add_event(aligned_linear_velocity_event);

        FP_DATA_TYPE aligned_linear_acceleration = (trig_buff->get_rot_mat(-rotation) * linear_acceleration).x();
        std::shared_ptr<IEvent<FP_DATA_TYPE>> aligned_linear_acceleration_event(new Event<FP_DATA_TYPE>(aligned_linear_acceleration_variable->get_full_name(), aligned_linear_acceleration, timestamp));
        aligned_linear_acceleration_variable->add_event(aligned_linear_acceleration_event);

        const FP_DATA_TYPE angular_velocity(state_data_entry["angular_velocity"].GetDouble());
        const FP_DATA_TYPE steer = angular_velocity / aligned_linear_velocity;
        const std::shared_ptr<IEvent<FP_DATA_TYPE>> steer_event(new Event<FP_DATA_TYPE>(steer_variable->get_full_name(), steer, timestamp));
        steer_variable->add_event(steer_event);
    }

    this->min_spatial_limits = geometry::Vec(min_position_x, min_position_y);
    this->max_spatial_limits = geometry::Vec(max_position_x, max_position_y);
}

std::string LyftDrivingAgent::get_name() const
{
    return this->name;
}

geometry::Vec LyftDrivingAgent::get_min_spatial_limits() const
{
    return this->min_spatial_limits;
}

geometry::Vec LyftDrivingAgent::get_max_spatial_limits() const
{
    return this->max_spatial_limits;
}

temporal::Time LyftDrivingAgent::get_min_temporal_limit() const
{
    return this->min_temporal_limit;
}

temporal::Time LyftDrivingAgent::get_max_temporal_limit() const
{
    return this->max_temporal_limit;
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> LyftDrivingAgent::get_constant_parameters() const
{
    return std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>>(
            new structures::stl::STLStackArray<std::shared_ptr<const IValuelessConstant>>(constant_dict.get_values()));
}

std::shared_ptr<const IValuelessConstant> LyftDrivingAgent::get_constant_parameter(const std::string& constant_name) const
{
    return constant_dict[constant_name];
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessVariable>>> LyftDrivingAgent::get_variable_parameters() const
{
    return std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessVariable>>>(
            new structures::stl::STLStackArray<std::shared_ptr<const IValuelessVariable>>(variable_dict.get_values()));
}

std::shared_ptr<const IValuelessVariable> LyftDrivingAgent::get_variable_parameter(const std::string& variable_name) const
{
    return variable_dict[variable_name];
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> LyftDrivingAgent::get_events() const
{
    std::shared_ptr<const structures::IArray<std::string>> variable_names = variable_dict.get_keys();

    std::shared_ptr<structures::stl::STLConcatArray<std::shared_ptr<const IValuelessEvent>>> events(
                new structures::stl::STLConcatArray<std::shared_ptr<const IValuelessEvent>>(variable_names->count()));

    size_t i;
    for(i = 0; i < variable_names->count(); ++i)
    {
        events->get_array(i) = variable_dict[(*variable_names)[i]]->get_valueless_events();
    }

    return events;
}

std::shared_ptr<IDrivingAgent> LyftDrivingAgent::driving_agent_deep_copy() const
{
    std::shared_ptr<LyftDrivingAgent> driving_agent(new LyftDrivingAgent());

    driving_agent->min_spatial_limits = this->get_min_spatial_limits();
    driving_agent->max_spatial_limits = this->get_max_spatial_limits();
    driving_agent->min_temporal_limit = this->get_min_temporal_limit();
    driving_agent->max_temporal_limit = this->get_max_temporal_limit();

    size_t i;

    std::shared_ptr<const structures::IArray<std::string>> constant_names = constant_dict.get_keys();
    for(i = 0; i < constant_names->count(); ++i)
    {
        driving_agent->constant_dict.update((*constant_names)[i], constant_dict[(*constant_names)[i]]->valueless_shallow_copy());
    }

    std::shared_ptr<const structures::IArray<std::string>> variable_names = variable_dict.get_keys();
    for(i = 0; i < variable_names->count(); ++i)
    {
        driving_agent->variable_dict.update((*variable_names)[i], variable_dict[(*variable_names)[i]]->valueless_deep_copy());
    }

    return driving_agent;
}

std::shared_ptr<const IDrivingAgentState> LyftDrivingAgent::get_driving_agent_state(temporal::Time time) const
{
    throw utils::NotImplementedException();
}

}
}
}
}
