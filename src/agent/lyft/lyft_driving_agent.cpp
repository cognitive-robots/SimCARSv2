
#include <ori/simcars/structures/stl/stl_concat_array.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/agent/basic_constant.hpp>
#include <ori/simcars/agent/basic_event.hpp>
#include <ori/simcars/agent/basic_variable.hpp>
#include <ori/simcars/agent/lyft/lyft_driving_agent.hpp>

#include <magic_enum.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{
namespace lyft
{

LyftDrivingAgent::LyftDrivingAgent() {}

LyftDrivingAgent::LyftDrivingAgent(IDrivingScene const *driving_scene, rapidjson::Value::ConstObject const &json_agent_data)
    : driving_scene(driving_scene)
{
    this->min_temporal_limit = temporal::Time::max();
    this->max_temporal_limit = temporal::Time::min();

    FP_DATA_TYPE min_position_x = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_position_x = std::numeric_limits<FP_DATA_TYPE>::min();
    FP_DATA_TYPE min_position_y = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_position_y = std::numeric_limits<FP_DATA_TYPE>::min();

    uint32_t const id = json_agent_data["id"].GetInt();
    bool const ego = json_agent_data["ego"].GetBool();

    this->name = (ego ? "ego_vehicle_" : "non_ego_vehicle_") + std::to_string(id);

    IConstant<uint32_t>* const id_constant = new BasicConstant<uint32_t>(this->name, "id", id);
    this->constant_dict.update(id_constant->get_full_name(), id_constant);

    IConstant<bool>* const ego_constant = new BasicConstant<bool>(this->name, "ego", ego);
    this->constant_dict.update(ego_constant->get_full_name(), ego_constant);

    rapidjson::Value::ConstArray const &bounding_box_data = json_agent_data["bounding_box"].GetArray();
    FP_DATA_TYPE const bb_length = bounding_box_data[0].GetDouble();
    FP_DATA_TYPE const bb_width = bounding_box_data[1].GetDouble();

    IConstant<FP_DATA_TYPE>* const bb_length_constant = new BasicConstant<FP_DATA_TYPE>(this->name, "bb_length", bb_length);
    this->constant_dict.update(bb_length_constant->get_full_name(), bb_length_constant);

    IConstant<FP_DATA_TYPE>* const bb_width_constant = new BasicConstant<FP_DATA_TYPE>(this->name, "bb_width", bb_width);
    this->constant_dict.update(bb_width_constant->get_full_name(), bb_width_constant);

    std::string class_label = json_agent_data["class_label"].GetString();
    size_t const first_underscore = class_label.find('_');
    size_t const second_underscore = class_label.find('_', first_underscore + 1);
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

    IConstant<DrivingAgentClass>* const driving_agent_class_constant = new BasicConstant<DrivingAgentClass>(this->name, "driving_agent_class", class_value);
    this->constant_dict.update(driving_agent_class_constant->get_full_name(), driving_agent_class_constant);


    rapidjson::Value::ConstArray const &state_data = json_agent_data["states"].GetArray();
    size_t const state_data_size = state_data.Capacity();

    IVariable<geometry::Vec>* const position_variable = new BasicVariable<geometry::Vec>(this->name, "position", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());
    this->variable_dict.update(position_variable->get_full_name(), position_variable);

    IVariable<geometry::Vec>* const linear_velocity_variable = new BasicVariable<geometry::Vec>(this->name, "linear_velocity", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());
    this->variable_dict.update(linear_velocity_variable->get_full_name(), linear_velocity_variable);

    IVariable<FP_DATA_TYPE>* const aligned_linear_velocity_variable = new BasicVariable<FP_DATA_TYPE>(this->name, "aligned_linear_velocity", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());
    this->variable_dict.update(aligned_linear_velocity_variable->get_full_name(), aligned_linear_velocity_variable);

    IVariable<geometry::Vec>* const linear_acceleration_variable = new BasicVariable<geometry::Vec>(this->name, "linear_acceleration", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());
    this->variable_dict.update(linear_acceleration_variable->get_full_name(), linear_acceleration_variable);

    IVariable<FP_DATA_TYPE>* const aligned_linear_acceleration_variable = new BasicVariable<FP_DATA_TYPE>(this->name, "aligned_linear_acceleration", IValuelessVariable::Type::INDIRECT_ACTUATION, this->get_scene()->get_time_step());
    this->variable_dict.update(aligned_linear_acceleration_variable->get_full_name(), aligned_linear_acceleration_variable);

    IVariable<geometry::Vec>* const external_linear_acceleration_variable = new BasicVariable<geometry::Vec>(this->name, "linear_acceleration", IValuelessVariable::Type::EXTERNAL, this->get_scene()->get_time_step());
    this->variable_dict.update(external_linear_acceleration_variable->get_full_name(), external_linear_acceleration_variable);

    IVariable<FP_DATA_TYPE>* const rotation_variable = new BasicVariable<FP_DATA_TYPE>(this->name, "rotation", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());
    this->variable_dict.update(rotation_variable->get_full_name(), rotation_variable);

    IVariable<FP_DATA_TYPE>* const steer_variable = new BasicVariable<FP_DATA_TYPE>(this->name, "steer", IValuelessVariable::Type::INDIRECT_ACTUATION, this->get_scene()->get_time_step());
    this->variable_dict.update(steer_variable->get_full_name(), steer_variable);

    IVariable<FP_DATA_TYPE>* const angular_velocity_variable = new BasicVariable<FP_DATA_TYPE>(this->name, "angular_velocity", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());
    this->variable_dict.update(angular_velocity_variable->get_full_name(), angular_velocity_variable);

    IVariable<temporal::Duration>* const ttc_variable = new BasicVariable<temporal::Duration>(this->name, "ttc", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());
    this->variable_dict.update(ttc_variable->get_full_name(), ttc_variable);

    IVariable<temporal::Duration>* const cumilative_collision_time_variable = new BasicVariable<temporal::Duration>(this->name, "cumilative_collision_time", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());
    this->variable_dict.update(cumilative_collision_time_variable->get_full_name(), cumilative_collision_time_variable);


    geometry::TrigBuff const *trig_buff = geometry::TrigBuff::get_instance();

    size_t i;
    for (i = 0; i < state_data_size; ++i)
    {
        rapidjson::Value::ConstObject const &state_data_entry = state_data[i].GetObject();
        temporal::Time const timestamp(temporal::Duration(state_data_entry["timestamp"].GetInt64()));
        this->min_temporal_limit = std::min(timestamp, this->min_temporal_limit);
        this->max_temporal_limit = std::max(timestamp, this->max_temporal_limit);

        rapidjson::Value::ConstArray const &position_data = state_data_entry["position"].GetArray();
        FP_DATA_TYPE const position_x = position_data[0].GetDouble();
        FP_DATA_TYPE const position_y = position_data[1].GetDouble();
        min_position_x = std::min(position_x, min_position_x);
        max_position_x = std::max(position_x, max_position_x);
        min_position_y = std::min(position_y, min_position_y);
        max_position_y = std::max(position_y, max_position_y);
        geometry::Vec const position(position_x, position_y);
        position_variable->set_value(timestamp, position);

        rapidjson::Value::ConstArray const &linear_velocity_data = state_data_entry["linear_velocity"].GetArray();
        geometry::Vec const linear_velocity(linear_velocity_data[0].GetDouble(), linear_velocity_data[1].GetDouble());
        linear_velocity_variable->set_value(timestamp, linear_velocity);

        rapidjson::Value::ConstArray const &linear_acceleration_data = state_data_entry["linear_acceleration"].GetArray();
        geometry::Vec const linear_acceleration(linear_acceleration_data[0].GetDouble(), linear_acceleration_data[1].GetDouble());
        linear_acceleration_variable->set_value(timestamp, linear_acceleration);

        FP_DATA_TYPE const rotation(state_data_entry["rotation"].GetDouble());
        rotation_variable->set_value(timestamp, rotation);

        FP_DATA_TYPE const aligned_linear_velocity = (trig_buff->get_rot_mat(-rotation) * linear_velocity).x();
        aligned_linear_velocity_variable->set_value(timestamp, aligned_linear_velocity);

        FP_DATA_TYPE const aligned_linear_acceleration = (trig_buff->get_rot_mat(-rotation) * linear_acceleration).x();
        aligned_linear_acceleration_variable->set_value(timestamp, aligned_linear_acceleration);

        geometry::Vec const external_linear_acceleration = linear_acceleration - (trig_buff->get_rot_mat(rotation) * geometry::Vec(aligned_linear_acceleration, 0));
        external_linear_acceleration_variable->set_value(timestamp, external_linear_acceleration);

        FP_DATA_TYPE const angular_velocity = state_data_entry["angular_velocity"].GetDouble();
        angular_velocity_variable->set_value(timestamp, angular_velocity);

        FP_DATA_TYPE const steer = angular_velocity / aligned_linear_velocity;
        steer_variable->set_value(timestamp, steer);

        // WARNING: Sets TTC to maximum, could potentially mess with reward and IDM calculations
        ttc_variable->set_value(timestamp, temporal::Duration::max());

        // WARNING: Assumes no collisions are present in the dataset
        cumilative_collision_time_variable->set_value(timestamp, temporal::Duration(0));
    }

    this->min_spatial_limits = geometry::Vec(min_position_x, min_position_y);
    this->max_spatial_limits = geometry::Vec(max_position_x, max_position_y);
}

LyftDrivingAgent::~LyftDrivingAgent()
{
    size_t i;

    structures::IArray<IValuelessConstant*> const *constants = constant_dict.get_values();

    for (i = 0; i < constants->count(); ++i)
    {
        delete (*constants)[i];
    }

    structures::IArray<IValuelessVariable*> const *variables = variable_dict.get_values();

    for (i = 0; i < variables->count(); ++i)
    {
        delete (*variables)[i];
    }
}

std::string LyftDrivingAgent::get_name() const
{
    return this->name;
}

IDrivingScene const* LyftDrivingAgent::get_driving_scene() const
{
    return this->driving_scene;
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

structures::IArray<IValuelessConstant const*>* LyftDrivingAgent::get_constant_parameters() const
{
    structures::stl::STLStackArray<IValuelessConstant const*> *constants =
            new structures::stl::STLStackArray<IValuelessConstant const*>(constant_dict.count());
    cast_array(*constant_dict.get_values(), *constants);
    return constants;
}

IValuelessConstant const* LyftDrivingAgent::get_constant_parameter(std::string const &constant_name) const
{
    if (constant_dict.contains(constant_name))
    {
        return constant_dict[constant_name];
    }
    else
    {
        return nullptr;
    }
}

structures::IArray<IValuelessVariable const*>* LyftDrivingAgent::get_variable_parameters() const
{
    structures::stl::STLStackArray<IValuelessVariable const*> *variables =
            new structures::stl::STLStackArray<IValuelessVariable const*>(variable_dict.count());
    cast_array(*variable_dict.get_values(), *variables);
    return variables;
}

IValuelessVariable const* LyftDrivingAgent::get_variable_parameter(std::string const &variable_name) const
{
    if (variable_dict.contains(variable_name))
    {
        return variable_dict[variable_name];
    }
    else
    {
        return nullptr;
    }
}

structures::IArray<IValuelessEvent const*>* LyftDrivingAgent::get_events() const
{
    structures::IArray<std::string> const *variable_names = variable_dict.get_keys();

    structures::stl::STLConcatArray<IValuelessEvent const*> *events =
            new structures::stl::STLConcatArray<IValuelessEvent const*>(variable_names->count());

    size_t i;
    for(i = 0; i < variable_names->count(); ++i)
    {
        events->get_array(i) = variable_dict[(*variable_names)[i]]->get_valueless_events();
    }

    return events;
}

IDrivingAgent* LyftDrivingAgent::driving_agent_deep_copy() const
{
    LyftDrivingAgent *driving_agent = new LyftDrivingAgent();

    driving_agent->min_spatial_limits = this->get_min_spatial_limits();
    driving_agent->max_spatial_limits = this->get_max_spatial_limits();
    driving_agent->min_temporal_limit = this->get_min_temporal_limit();
    driving_agent->max_temporal_limit = this->get_max_temporal_limit();

    size_t i;

    structures::IArray<std::string> const *constant_names = constant_dict.get_keys();
    for(i = 0; i < constant_names->count(); ++i)
    {
        driving_agent->constant_dict.update((*constant_names)[i], constant_dict[(*constant_names)[i]]->valueless_constant_shallow_copy());
    }

    structures::IArray<std::string> const *variable_names = variable_dict.get_keys();
    for(i = 0; i < variable_names->count(); ++i)
    {
        driving_agent->variable_dict.update((*variable_names)[i], variable_dict[(*variable_names)[i]]->valueless_deep_copy());
    }

    return driving_agent;
}

structures::IArray<IValuelessConstant*>* LyftDrivingAgent::get_mutable_constant_parameters()
{
    structures::stl::STLStackArray<IValuelessConstant*> *constants =
            new structures::stl::STLStackArray<IValuelessConstant*>(constant_dict.get_values());
    constant_dict.get_values(constants);
    return constants;
}

IValuelessConstant* LyftDrivingAgent::get_mutable_constant_parameter(std::string const &constant_name)
{
    if (constant_dict.contains(constant_name))
    {
        return constant_dict[constant_name];
    }
    else
    {
        return nullptr;
    }
}

structures::IArray<IValuelessVariable*>* LyftDrivingAgent::get_mutable_variable_parameters()
{
    structures::stl::STLStackArray<IValuelessVariable*> *variables =
            new structures::stl::STLStackArray<IValuelessVariable*>(variable_dict.count());
    variable_dict.get_values(variables);
    return variables;
}

IValuelessVariable* LyftDrivingAgent::get_mutable_variable_parameter(std::string const &variable_name)
{
    if (variable_dict.contains(variable_name))
    {
        return variable_dict[variable_name];
    }
    else
    {
        return nullptr;
    }
}

structures::IArray<IValuelessEvent*>* LyftDrivingAgent::get_mutable_events()
{
    structures::IArray<std::string> const *variable_names = variable_dict.get_keys();

    structures::stl::STLConcatArray<IValuelessEvent*> *events =
            new structures::stl::STLConcatArray<IValuelessEvent*>(variable_names->count());

    size_t i;
    for(i = 0; i < variable_names->count(); ++i)
    {
        events->get_array(i) = variable_dict[(*variable_names)[i]]->get_mutable_valueless_events();
    }

    return events;
}

}
}
}
}
