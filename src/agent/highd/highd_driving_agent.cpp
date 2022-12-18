
#include <ori/simcars/structures/stl/stl_concat_array.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/agent/highd/highd_driving_agent.hpp>
#include <ori/simcars/agent/basic_constant.hpp>
#include <ori/simcars/agent/basic_event.hpp>
#include <ori/simcars/agent/basic_variable.hpp>

#include <cassert>

namespace ori
{
namespace simcars
{
namespace agent
{
namespace highd
{

HighDDrivingAgent::HighDDrivingAgent() {}

HighDDrivingAgent::HighDDrivingAgent(IDrivingScene const *driving_scene,
                                     size_t tracks_meta_row,
                                     const rapidcsv::Document &tracks_meta_csv_document,
                                     const rapidcsv::Document &tracks_csv_document)
    : driving_scene(driving_scene)
{
    size_t const start_frame = tracks_meta_csv_document.GetCell<size_t>("initialFrame", tracks_meta_row) - 1;
    size_t const end_frame = tracks_meta_csv_document.GetCell<size_t>("finalFrame", tracks_meta_row) - 1;

    // WARNING: This doesn't actually set the min and max temporal limits to the real world time points, based upon ticks since
    // epoch, in theory this is possible, but unnecessary at this stage
    this->min_temporal_limit = temporal::Time(temporal::Duration(start_frame * 40));
    this->max_temporal_limit = temporal::Time(temporal::Duration(end_frame * 40));

    FP_DATA_TYPE min_position_x = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_position_x = std::numeric_limits<FP_DATA_TYPE>::min();
    FP_DATA_TYPE min_position_y = std::numeric_limits<FP_DATA_TYPE>::max();
    FP_DATA_TYPE max_position_y = std::numeric_limits<FP_DATA_TYPE>::min();

    uint32_t const id = tracks_meta_csv_document.GetCell<uint32_t>("id", tracks_meta_row);
    bool const ego = false;

    this->name = (ego ? "ego_vehicle_" : "non_ego_vehicle_") + std::to_string(id);

    IConstant<uint32_t>* const id_constant = new BasicConstant<uint32_t>(this->name, "id", id);
    this->constant_dict.update(id_constant->get_full_name(), id_constant);

    IConstant<bool>* const ego_constant = new BasicConstant<bool>(this->name, "ego", ego);
    this->constant_dict.update(ego_constant->get_full_name(), ego_constant);

    FP_DATA_TYPE const bb_length = tracks_meta_csv_document.GetCell<FP_DATA_TYPE>("width", tracks_meta_row);
    FP_DATA_TYPE const bb_width = tracks_meta_csv_document.GetCell<FP_DATA_TYPE>("height", tracks_meta_row);

    IConstant<FP_DATA_TYPE>* const bb_length_constant = new BasicConstant<FP_DATA_TYPE>(this->name, "bb_length", bb_length);
    this->constant_dict.update(bb_length_constant->get_full_name(), bb_length_constant);

    IConstant<FP_DATA_TYPE>* const bb_width_constant = new BasicConstant<FP_DATA_TYPE>(this->name, "bb_width", bb_width);
    this->constant_dict.update(bb_width_constant->get_full_name(), bb_width_constant);

    std::string class_label = tracks_meta_csv_document.GetCell<std::string>("class", tracks_meta_row);
    DrivingAgentClass class_value;
    if (class_label == "Car")
    {
        class_value = DrivingAgentClass::CAR;
    }
    else if (class_label == "Truck")
    {
        class_value = DrivingAgentClass::TRUCK;
    }
    else
    {
        class_value = DrivingAgentClass::UNKNOWN;
    }

    IConstant<DrivingAgentClass>* const driving_agent_class_constant = new BasicConstant<DrivingAgentClass>(this->name, "driving_agent_class", class_value);
    this->constant_dict.update(driving_agent_class_constant->get_full_name(), driving_agent_class_constant);


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

    for (i = 0; i < tracks_csv_document.GetRowCount(); ++i)
    {
        if (tracks_csv_document.GetCell<uint32_t>("id", i) == id)
        {
            break;
        }
    }
    if (i == tracks_csv_document.GetRowCount())
    {
        throw std::runtime_error("Could not find agent id specified in tracks meta file in tracks file");
    }

    size_t const tracks_start_row = i;
    size_t const tracks_end_row = tracks_start_row + (end_frame - start_frame);
    for (i = tracks_start_row; i <= tracks_end_row; ++i)
    {
        temporal::Time timestamp = this->min_temporal_limit + temporal::Duration((i - tracks_start_row) * 40);
        assert(timestamp <= this->max_temporal_limit);

        FP_DATA_TYPE const position_x = tracks_csv_document.GetCell<FP_DATA_TYPE>("x", i) + bb_length / 2.0f;
        FP_DATA_TYPE const position_y = tracks_csv_document.GetCell<FP_DATA_TYPE>("y", i) + bb_width / 2.0f;
        min_position_x = std::min(position_x, min_position_x);
        max_position_x = std::max(position_x, max_position_x);
        min_position_y = std::min(position_y, min_position_y);
        max_position_y = std::max(position_y, max_position_y);
        geometry::Vec const position(position_x, position_y);
        position_variable->set_value(timestamp, position);

        geometry::Vec const linear_velocity = geometry::Vec(tracks_csv_document.GetCell<FP_DATA_TYPE>("xVelocity", i),
                                                            tracks_csv_document.GetCell<FP_DATA_TYPE>("yVelocity", i)) * 1e-3f;
        linear_velocity_variable->set_value(timestamp, linear_velocity);

        geometry::Vec const linear_acceleration = geometry::Vec(tracks_csv_document.GetCell<FP_DATA_TYPE>("xAcceleration", i),
                                                                tracks_csv_document.GetCell<FP_DATA_TYPE>("yAcceleration", i)) * 1e-6f;
        linear_acceleration_variable->set_value(timestamp, linear_acceleration);

        // WARNING: This makes a rather big assumption that the vehicles are always driving parallel to the lanes, mainly
        // because the dataset doesn't actually provide any orientation information
        FP_DATA_TYPE rotation;
        uint32_t driving_direction = tracks_meta_csv_document.GetCell<uint32_t>("drivingDirection", tracks_meta_row);
        if (driving_direction == 1)
        {
            rotation = -M_PI;
        }
        else if (driving_direction == 2)
        {
            rotation = 0.0f;
        }
        else
        {
            throw std::runtime_error("Unrecognised driving direction");
        }
        rotation_variable->set_value(timestamp, rotation);

        FP_DATA_TYPE const aligned_linear_velocity = (trig_buff->get_rot_mat(-rotation) * linear_velocity).x();
        aligned_linear_velocity_variable->set_value(timestamp, aligned_linear_velocity);

        FP_DATA_TYPE const aligned_linear_acceleration = (trig_buff->get_rot_mat(-rotation) * linear_acceleration).x();
        aligned_linear_acceleration_variable->set_value(timestamp, aligned_linear_acceleration);

        geometry::Vec const external_linear_acceleration = linear_acceleration - (trig_buff->get_rot_mat(rotation) * geometry::Vec(aligned_linear_acceleration, 0));
        external_linear_acceleration_variable->set_value(timestamp, external_linear_acceleration);

        // WARNING: Again, assuming no angular velocity due to lack of orientation information
        FP_DATA_TYPE const angular_velocity = 0.0f;
        angular_velocity_variable->set_value(timestamp, angular_velocity);

        FP_DATA_TYPE const steer = angular_velocity / aligned_linear_velocity;
        steer_variable->set_value(timestamp, steer);

        FP_DATA_TYPE const ttc_raw = tracks_csv_document.GetCell<FP_DATA_TYPE>("ttc", i);
        temporal::Duration ttc;
        if (ttc_raw > 0.0f)
        {
            ttc = temporal::Duration(int64_t(ttc_raw * 1e+3f));
        }
        else
        {
            ttc = temporal::Duration::max();
        }
        ttc_variable->set_value(timestamp, ttc);

        // WARNING: Assumes no collisions are present in the dataset
        cumilative_collision_time_variable->set_value(timestamp, temporal::Duration(0));
    }

    this->min_spatial_limits = geometry::Vec(min_position_x, min_position_y);
    this->max_spatial_limits = geometry::Vec(max_position_x, max_position_y);
}

HighDDrivingAgent::~HighDDrivingAgent()
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

std::string HighDDrivingAgent::get_name() const
{
    return this->name;
}

IDrivingScene const* HighDDrivingAgent::get_driving_scene() const
{
    return this->driving_scene;
}

geometry::Vec HighDDrivingAgent::get_min_spatial_limits() const
{
    return this->min_spatial_limits;
}

geometry::Vec HighDDrivingAgent::get_max_spatial_limits() const
{
    return this->max_spatial_limits;
}

temporal::Time HighDDrivingAgent::get_min_temporal_limit() const
{
    return this->min_temporal_limit;
}

temporal::Time HighDDrivingAgent::get_max_temporal_limit() const
{
    return this->max_temporal_limit;
}

structures::IArray<IValuelessConstant const*>* HighDDrivingAgent::get_constant_parameters() const
{
    structures::stl::STLStackArray<IValuelessConstant const*> *constants =
            new structures::stl::STLStackArray<IValuelessConstant const*>(constant_dict.count());
    cast_array(*constant_dict.get_values(), *constants);
    return constants;
}

IValuelessConstant const* HighDDrivingAgent::get_constant_parameter(std::string const &constant_name) const
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

structures::IArray<IValuelessVariable const*>* HighDDrivingAgent::get_variable_parameters() const
{
    structures::stl::STLStackArray<IValuelessVariable const*> *variables =
            new structures::stl::STLStackArray<IValuelessVariable const*>(variable_dict.count());
    cast_array(*variable_dict.get_values(), *variables);
    return variables;
}

IValuelessVariable const* HighDDrivingAgent::get_variable_parameter(std::string const &variable_name) const
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

structures::IArray<IValuelessEvent const*>* HighDDrivingAgent::get_events() const
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

IDrivingAgent* HighDDrivingAgent::driving_agent_deep_copy(IDrivingScene *driving_scene) const
{
    HighDDrivingAgent *driving_agent = new HighDDrivingAgent();

    driving_agent->name = this->name;

    driving_agent->min_spatial_limits = this->min_spatial_limits;
    driving_agent->max_spatial_limits = this->max_spatial_limits;
    driving_agent->min_temporal_limit = this->min_temporal_limit;
    driving_agent->max_temporal_limit = this->max_temporal_limit;

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

    if (driving_scene == nullptr)
    {
        driving_agent->driving_scene = this->driving_scene;
    }
    else
    {
        driving_agent->driving_scene = driving_scene;
    }

    return driving_agent;
}

structures::IArray<IValuelessConstant*>* HighDDrivingAgent::get_mutable_constant_parameters()
{
    structures::stl::STLStackArray<IValuelessConstant*> *constants =
            new structures::stl::STLStackArray<IValuelessConstant*>(constant_dict.get_values());
    constant_dict.get_values(constants);
    return constants;
}

IValuelessConstant* HighDDrivingAgent::get_mutable_constant_parameter(std::string const &constant_name)
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

structures::IArray<IValuelessVariable*>* HighDDrivingAgent::get_mutable_variable_parameters()
{
    structures::stl::STLStackArray<IValuelessVariable*> *variables =
            new structures::stl::STLStackArray<IValuelessVariable*>(variable_dict.count());
    variable_dict.get_values(variables);
    return variables;
}

IValuelessVariable* HighDDrivingAgent::get_mutable_variable_parameter(std::string const &variable_name)
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

structures::IArray<IValuelessEvent*>* HighDDrivingAgent::get_mutable_events()
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
