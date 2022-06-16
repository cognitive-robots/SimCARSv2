
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/constant.hpp>
#include <ori/simcars/agent/event.hpp>
#include <ori/simcars/agent/variable.hpp>
#include <ori/simcars/agent/entity.hpp>
#include <ori/simcars/agent/lyft/lyft_scene.hpp>

#include <laudrup/lz4_stream/lz4_stream.hpp>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <neargye/magic_enum/magic_enum.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{
namespace lyft
{

std::ostream& operator <<(std::ostream& output_stream, const RoadAgentClass& road_agent_class)
{
    return output_stream << magic_enum::enum_name(road_agent_class);
}

std::shared_ptr<const LyftScene> LyftScene::construct_from(std::shared_ptr<const IScene> scene)
{
    std::shared_ptr<LyftScene> new_scene(new LyftScene());

    new_scene->min_spatial_limits = scene->get_min_spatial_limits();
    new_scene->max_spatial_limits = scene->get_max_spatial_limits();
    new_scene->min_temporal_limit = scene->get_min_temporal_limit();
    new_scene->max_temporal_limit = scene->get_max_temporal_limit();

    std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> entities = scene->get_entities();

    size_t i;
    for(i = 0; i < entities->count(); ++i)
    {
        std::shared_ptr<IEntity> new_entity = (*entities)[i]->deep_copy();

        new_scene->entity_dict.update(new_entity->get_name(), new_entity);
    }

    return new_scene;
}

void LyftScene::save_virt(std::ofstream &output_filestream) const
{
    throw utils::NotImplementedException();
}

void LyftScene::load_virt(std::ifstream& input_filestream)
{
    lz4_stream::istream input_lz4_stream(input_filestream);
    rapidjson::BasicIStreamWrapper input_lz4_json_stream(input_lz4_stream);

    rapidjson::Document json_document;
    json_document.ParseStream(input_lz4_json_stream);

    min_temporal_limit = temporal::Time::max();
    max_temporal_limit = temporal::Time::min();

    FP_DATA_TYPE min_position_x = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_position_x = std::numeric_limits<FP_DATA_TYPE>::min();
    FP_DATA_TYPE min_position_y = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_position_y = std::numeric_limits<FP_DATA_TYPE>::min();

    for (const rapidjson::Value& json_document_element : json_document.GetArray())
    {
        const rapidjson::Value::ConstObject& json_agent_data = json_document_element.GetObject();

        const uint32_t id = json_agent_data["id"].GetInt();
        const bool ego = json_agent_data["ego"].GetBool();

        const std::string entity_name = (ego ? "ego_vehicle_" : "non_ego_vehicle_") + std::to_string(id);

        const std::shared_ptr<IEntity> driving_agent(new Entity(entity_name));
        entity_dict.update(entity_name, driving_agent);

        const std::shared_ptr<IConstant<uint32_t>> id_constant(new Constant<uint32_t>(entity_name, "id", id));
        driving_agent->add_constant_parameter(id_constant->get_full_name(), id_constant);

        const std::shared_ptr<IConstant<bool>> ego_constant(new Constant<bool>(entity_name, "ego", ego));
        driving_agent->add_constant_parameter(ego_constant->get_full_name(), ego_constant);

        const rapidjson::Value::ConstArray bounding_box_data = json_agent_data["bounding_box"].GetArray();
        const FP_DATA_TYPE bb_length = bounding_box_data[0].GetDouble();
        const FP_DATA_TYPE bb_width = bounding_box_data[1].GetDouble();

        const std::shared_ptr<IConstant<FP_DATA_TYPE>> bb_length_constant(new Constant<FP_DATA_TYPE>(entity_name, "bb_length", bb_length));
        driving_agent->add_constant_parameter(bb_length_constant->get_full_name(), bb_length_constant);

        const std::shared_ptr<IConstant<FP_DATA_TYPE>> bb_width_constant(new Constant<FP_DATA_TYPE>(entity_name, "bb_width", bb_width));
        driving_agent->add_constant_parameter(bb_width_constant->get_full_name(), bb_width_constant);

        std::string class_label = json_agent_data["class_label"].GetString();
        const size_t first_underscore = class_label.find('_');
        const size_t second_underscore = class_label.find('_', first_underscore + 1);
        class_label = class_label.substr(second_underscore + 1);
        auto temp_class_value = magic_enum::enum_cast<RoadAgentClass>(class_label);
        RoadAgentClass class_value;
        if (temp_class_value.has_value())
        {
            class_value = temp_class_value.value();
        }
        else
        {
            class_value = RoadAgentClass::UNKNOWN;
        }

        const std::shared_ptr<IConstant<RoadAgentClass>> road_agent_class_constant(new Constant<RoadAgentClass>(entity_name, "road_agent_class", class_value));
        driving_agent->add_constant_parameter(road_agent_class_constant->get_full_name(), road_agent_class_constant);

        const rapidjson::Value::ConstArray state_data = json_agent_data["states"].GetArray();
        const size_t state_data_size = state_data.Capacity();

        const std::shared_ptr<IVariable<geometry::Vec>> position_variable(new Variable<geometry::Vec>(entity_name, "position", IValuelessVariable::Type::BASE));
        driving_agent->add_variable_parameter(position_variable->get_full_name(), position_variable);

        const std::shared_ptr<IVariable<geometry::Vec>> linear_velocity_variable(new Variable<geometry::Vec>(entity_name, "linear_velocity", IValuelessVariable::Type::BASE));
        driving_agent->add_variable_parameter(linear_velocity_variable->get_full_name(), linear_velocity_variable);

        const std::shared_ptr<IVariable<geometry::Vec>> linear_acceleration_variable(new Variable<geometry::Vec>(entity_name, "linear_acceleration", IValuelessVariable::Type::INDIRECT_ACTUATION));
        driving_agent->add_variable_parameter(linear_acceleration_variable->get_full_name(), linear_acceleration_variable);

        const std::shared_ptr<IVariable<FP_DATA_TYPE>> rotation_variable(new Variable<FP_DATA_TYPE>(entity_name, "rotation", IValuelessVariable::Type::BASE));
        driving_agent->add_variable_parameter(rotation_variable->get_full_name(), rotation_variable);

        const std::shared_ptr<IVariable<FP_DATA_TYPE>> steer_variable(new Variable<FP_DATA_TYPE>(entity_name, "steer", IValuelessVariable::Type::INDIRECT_ACTUATION));
        driving_agent->add_variable_parameter(steer_variable->get_full_name(), steer_variable);

        //const std::shared_ptr<IVariable<FP_DATA_TYPE>> rate_of_steer_variable(new Variable<FP_DATA_TYPE>(entity_name, "rate_of_steer", IValuelessVariable::Type::INDIRECT_ACTUATION));
        //driving_agent->add_variable_parameter(rate_of_steer_variable->get_full_name(), rate_of_steer_variable);

        //const std::shared_ptr<IVariable<std::string>> lane_id_variable(new Variable<std::string>(entity_name, "lane_id", IValuelessVariable::Type::BASE));
        //driving_agent->add_variable_parameter(lane_id_variable->get_full_name(), lane_id_variable);

        size_t i;
        for (i = 0; i < state_data_size; ++i)
        {
            const rapidjson::Value::ConstObject state_data_entry = state_data[i].GetObject();
            const temporal::Time timestamp(temporal::Duration(state_data_entry["timestamp"].GetInt64()));
            min_temporal_limit = std::min(timestamp, min_temporal_limit);
            max_temporal_limit = std::max(timestamp, max_temporal_limit);

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

            // Note: Should probably only use component parallel to orientation of the vehicle.
            const FP_DATA_TYPE angular_velocity(state_data_entry["angular_velocity"].GetDouble());
            const FP_DATA_TYPE steer = angular_velocity / linear_velocity.norm();
            const std::shared_ptr<IEvent<FP_DATA_TYPE>> steer_event(new Event<FP_DATA_TYPE>(steer_variable->get_full_name(), steer, timestamp));
            steer_variable->add_event(steer_event);

            //const FP_DATA_TYPE angular_acceleration(state_data_entry["angular_acceleration"].GetDouble());
            //const FP_DATA_TYPE rate_of_steer = (angular_acceleration - (steer * linear_acceleration.norm())) / linear_velocity.norm();
            //const std::shared_ptr<IEvent<FP_DATA_TYPE>> rate_of_steer_event(new Event<FP_DATA_TYPE>(rate_of_steer_variable->get_full_name(), rate_of_steer, timestamp));
            //rate_of_steer_variable->add_event(rate_of_steer_event);

            //const std::string lane_id(state_data_entry["lane"].GetString());
            //const std::shared_ptr<IEvent<std::string>> lane_id_event(new Event<std::string>(lane_id_variable->get_full_name(), lane_id, timestamp));
            //lane_id_variable->add_event(lane_id_event);
        }
    }

    min_spatial_limits = geometry::Vec(min_position_x, min_position_y);
    max_spatial_limits = geometry::Vec(max_position_x, max_position_y);
}

geometry::Vec LyftScene::get_min_spatial_limits() const
{
    return this->min_spatial_limits;
}

geometry::Vec LyftScene::get_max_spatial_limits() const
{
    return this->max_spatial_limits;
}

temporal::Time LyftScene::get_min_temporal_limit() const
{
    return this->min_temporal_limit;
}

temporal::Time LyftScene::get_max_temporal_limit() const
{
    return this->max_temporal_limit;
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> LyftScene::get_entities() const
{
    return std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>>(
            new structures::stl::STLStackArray<std::shared_ptr<const IEntity>>(entity_dict.get_values()));
}

std::shared_ptr<const IEntity> LyftScene::get_entity(const std::string& entity_name) const
{
    return entity_dict[entity_name];
}

}
}
}
}
