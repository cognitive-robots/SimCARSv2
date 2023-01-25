
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
    size_t const start_frame = tracks_meta_csv_document.GetCell<size_t>("initialFrame", tracks_meta_row);
    size_t const end_frame = tracks_meta_csv_document.GetCell<size_t>("finalFrame", tracks_meta_row);

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

    id_constant = new BasicConstant<uint32_t>(this->name, "id", id);

    ego_constant = new BasicConstant<bool>(this->name, "ego", ego);

    FP_DATA_TYPE const bb_length = tracks_meta_csv_document.GetCell<FP_DATA_TYPE>("width", tracks_meta_row);
    FP_DATA_TYPE const bb_width = tracks_meta_csv_document.GetCell<FP_DATA_TYPE>("height", tracks_meta_row);

    bb_length_constant = new BasicConstant<FP_DATA_TYPE>(this->name, "bb_length", bb_length);

    bb_width_constant = new BasicConstant<FP_DATA_TYPE>(this->name, "bb_width", bb_width);

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

    driving_agent_class_constant = new BasicConstant<DrivingAgentClass>(this->name, "driving_agent_class", class_value);


    position_variable = new BasicVariable<geometry::Vec>(this->name, "position", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());

    linear_velocity_variable = new BasicVariable<geometry::Vec>(this->name, "linear_velocity", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());

    aligned_linear_velocity_variable = new BasicVariable<FP_DATA_TYPE>(this->name, "aligned_linear_velocity", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());

    linear_acceleration_variable = new BasicVariable<geometry::Vec>(this->name, "linear_acceleration", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());

    aligned_linear_acceleration_variable = new BasicVariable<FP_DATA_TYPE>(this->name, "aligned_linear_acceleration", IValuelessVariable::Type::INDIRECT_ACTUATION, this->get_scene()->get_time_step());

    external_linear_acceleration_variable = new BasicVariable<geometry::Vec>(this->name, "linear_acceleration", IValuelessVariable::Type::EXTERNAL, this->get_scene()->get_time_step());

    rotation_variable = new BasicVariable<FP_DATA_TYPE>(this->name, "rotation", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());

    steer_variable = new BasicVariable<FP_DATA_TYPE>(this->name, "steer", IValuelessVariable::Type::INDIRECT_ACTUATION, this->get_scene()->get_time_step());

    angular_velocity_variable = new BasicVariable<FP_DATA_TYPE>(this->name, "angular_velocity", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());

    ttc_variable = new BasicVariable<temporal::Duration>(this->name, "ttc", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());

    cumilative_collision_time_variable = new BasicVariable<temporal::Duration>(this->name, "cumilative_collision_time", IValuelessVariable::Type::BASE, this->get_scene()->get_time_step());


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
    delete id_constant;
    delete ego_constant;
    delete bb_length_constant;
    delete bb_width_constant;
    delete driving_agent_class_constant;

    delete position_variable;
    delete linear_velocity_variable;
    delete aligned_linear_velocity_variable;
    delete linear_acceleration_variable;
    delete aligned_linear_acceleration_variable;
    delete external_linear_acceleration_variable;
    delete rotation_variable;
    delete steer_variable;
    delete angular_velocity_variable;
    delete ttc_variable;
    delete cumilative_collision_time_variable;
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
    return new structures::stl::STLStackArray<IValuelessConstant const*>(
    {
                    id_constant,
                    ego_constant,
                    bb_length_constant,
                    bb_width_constant,
                    driving_agent_class_constant
                });
}

IValuelessConstant const* HighDDrivingAgent::get_constant_parameter(std::string const &constant_name) const
{
    if (constant_name == this->get_name() + ".id")
    {
        return id_constant;
    }
    else if (constant_name == this->get_name() + ".ego")
    {
        return ego_constant;
    }
    else if (constant_name == this->get_name() + ".bb_length")
    {
        return bb_length_constant;
    }
    else if (constant_name == this->get_name() + ".bb_width")
    {
        return bb_width_constant;
    }
    else if (constant_name == this->get_name() + ".driving_agent_class")
    {
        return driving_agent_class_constant;
    }
    else
    {
        return nullptr;
    }
}

structures::IArray<IValuelessVariable const*>* HighDDrivingAgent::get_variable_parameters() const
{
    return new structures::stl::STLStackArray<IValuelessVariable const*>(
    {
                    position_variable,
                    linear_velocity_variable,
                    aligned_linear_velocity_variable,
                    linear_acceleration_variable,
                    aligned_linear_acceleration_variable,
                    external_linear_acceleration_variable,
                    rotation_variable,
                    steer_variable,
                    angular_velocity_variable,
                    ttc_variable,
                    cumilative_collision_time_variable
                });
}

IValuelessVariable const* HighDDrivingAgent::get_variable_parameter(std::string const &variable_name) const
{
    if (variable_name == this->get_name() + ".position.base")
    {
        return position_variable;
    }
    else if (variable_name == this->get_name() + ".linear_velocity.base")
    {
        return linear_velocity_variable;
    }
    else if (variable_name == this->get_name() + ".aligned_linear_velocity.base")
    {
        return aligned_linear_velocity_variable;
    }
    else if (variable_name == this->get_name() + ".linear_acceleration.base")
    {
        return linear_acceleration_variable;
    }
    else if (variable_name == this->get_name() + ".aligned_linear_acceleration.indirect_actuation")
    {
        return aligned_linear_acceleration_variable;
    }
    else if (variable_name == this->get_name() + ".linear_acceleration.external")
    {
        return external_linear_acceleration_variable;
    }
    else if (variable_name == this->get_name() + ".rotation.base")
    {
        return rotation_variable;
    }
    else if (variable_name == this->get_name() + ".steer.indirect_actuation")
    {
        return steer_variable;
    }
    else if (variable_name == this->get_name() + ".angular_velocity.base")
    {
        return angular_velocity_variable;
    }
    else if (variable_name == this->get_name() + ".ttc.base")
    {
        return ttc_variable;
    }
    else if (variable_name == this->get_name() + ".cumilative_collision_time.base")
    {
        return cumilative_collision_time_variable;
    }
    else
    {
        return nullptr;
    }
}

structures::IArray<IValuelessEvent const*>* HighDDrivingAgent::get_events() const
{
    structures::stl::STLConcatArray<IValuelessEvent const*> *events =
            new structures::stl::STLConcatArray<IValuelessEvent const*>(11);

    events->get_array(0) = position_variable->get_valueless_events();
    events->get_array(1) = linear_velocity_variable->get_valueless_events();
    events->get_array(2) = aligned_linear_velocity_variable->get_valueless_events();
    events->get_array(3) = linear_acceleration_variable->get_valueless_events();
    events->get_array(4) = aligned_linear_acceleration_variable->get_valueless_events();
    events->get_array(5) = external_linear_acceleration_variable->get_valueless_events();
    events->get_array(6) = rotation_variable->get_valueless_events();
    events->get_array(7) = steer_variable->get_valueless_events();
    events->get_array(8) = angular_velocity_variable->get_valueless_events();
    events->get_array(9) = ttc_variable->get_valueless_events();
    events->get_array(10) = cumilative_collision_time_variable->get_valueless_events();

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

    driving_agent->id_constant = this->id_constant->constant_shallow_copy();
    driving_agent->ego_constant = this->ego_constant->constant_shallow_copy();
    driving_agent->bb_length_constant = this->bb_length_constant->constant_shallow_copy();
    driving_agent->bb_width_constant = this->bb_width_constant->constant_shallow_copy();
    driving_agent->driving_agent_class_constant = this->driving_agent_class_constant->constant_shallow_copy();

    driving_agent->position_variable = this->position_variable->variable_deep_copy();
    driving_agent->linear_velocity_variable = this->linear_velocity_variable->variable_deep_copy();
    driving_agent->aligned_linear_velocity_variable = this->aligned_linear_velocity_variable->variable_deep_copy();
    driving_agent->linear_acceleration_variable = this->linear_acceleration_variable->variable_deep_copy();
    driving_agent->aligned_linear_acceleration_variable = this->aligned_linear_acceleration_variable->variable_deep_copy();
    driving_agent->external_linear_acceleration_variable = this->external_linear_acceleration_variable->variable_deep_copy();
    driving_agent->rotation_variable = this->rotation_variable->variable_deep_copy();
    driving_agent->steer_variable = this->steer_variable->variable_deep_copy();
    driving_agent->angular_velocity_variable = this->angular_velocity_variable->variable_deep_copy();
    driving_agent->ttc_variable = this->ttc_variable->variable_deep_copy();
    driving_agent->cumilative_collision_time_variable = this->cumilative_collision_time_variable->variable_deep_copy();

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

IConstant<uint32_t> const* HighDDrivingAgent::get_id_constant() const
{
    return id_constant;
}

IConstant<bool> const* HighDDrivingAgent::get_ego_constant() const
{
    return ego_constant;
}

IConstant<FP_DATA_TYPE> const* HighDDrivingAgent::get_bb_length_constant() const
{
    return bb_length_constant;
}

IConstant<FP_DATA_TYPE> const* HighDDrivingAgent::get_bb_width_constant() const
{
    return bb_width_constant;
}

IConstant<DrivingAgentClass> const* HighDDrivingAgent::get_driving_agent_class_constant() const
{
    return driving_agent_class_constant;
}

IVariable<geometry::Vec> const* HighDDrivingAgent::get_position_variable() const
{
    return position_variable;
}

IVariable<geometry::Vec> const* HighDDrivingAgent::get_linear_velocity_variable() const
{
    return linear_velocity_variable;
}

IVariable<FP_DATA_TYPE> const* HighDDrivingAgent::get_aligned_linear_velocity_variable() const
{
    return aligned_linear_velocity_variable;
}

IVariable<geometry::Vec> const* HighDDrivingAgent::get_linear_acceleration_variable() const
{
    return linear_acceleration_variable;
}

IVariable<FP_DATA_TYPE> const* HighDDrivingAgent::get_aligned_linear_acceleration_variable() const
{
    return aligned_linear_acceleration_variable;
}

IVariable<geometry::Vec> const* HighDDrivingAgent::get_external_linear_acceleration_variable() const
{
    return external_linear_acceleration_variable;
}

IVariable<FP_DATA_TYPE> const* HighDDrivingAgent::get_rotation_variable() const
{
    return rotation_variable;
}

IVariable<FP_DATA_TYPE> const* HighDDrivingAgent::get_steer_variable() const
{
    return steer_variable;
}

IVariable<FP_DATA_TYPE> const* HighDDrivingAgent::get_angular_velocity_variable() const
{
    return angular_velocity_variable;
}

IVariable<temporal::Duration> const* HighDDrivingAgent::get_ttc_variable() const
{
    return ttc_variable;
}

IVariable<temporal::Duration> const* HighDDrivingAgent::get_cumilative_collision_time_variable() const
{
    return cumilative_collision_time_variable;
}

structures::IArray<IValuelessConstant*>* HighDDrivingAgent::get_mutable_constant_parameters()
{
    return new structures::stl::STLStackArray<IValuelessConstant*>(
    {
                    id_constant,
                    ego_constant,
                    bb_length_constant,
                    bb_width_constant,
                    driving_agent_class_constant
                });
}

IValuelessConstant* HighDDrivingAgent::get_mutable_constant_parameter(std::string const &constant_name)
{
    if (constant_name == this->get_name() + ".id")
    {
        return id_constant;
    }
    else if (constant_name == this->get_name() + ".ego")
    {
        return ego_constant;
    }
    else if (constant_name == this->get_name() + ".bb_length")
    {
        return bb_length_constant;
    }
    else if (constant_name == this->get_name() + ".bb_width")
    {
        return bb_width_constant;
    }
    else if (constant_name == this->get_name() + ".driving_agent_class")
    {
        return driving_agent_class_constant;
    }
    else
    {
        return nullptr;
    }
}

structures::IArray<IValuelessVariable*>* HighDDrivingAgent::get_mutable_variable_parameters()
{
    return new structures::stl::STLStackArray<IValuelessVariable*>(
    {
                    position_variable,
                    linear_velocity_variable,
                    aligned_linear_velocity_variable,
                    linear_acceleration_variable,
                    aligned_linear_acceleration_variable,
                    external_linear_acceleration_variable,
                    rotation_variable,
                    steer_variable,
                    angular_velocity_variable,
                    ttc_variable,
                    cumilative_collision_time_variable
                });
}

IValuelessVariable* HighDDrivingAgent::get_mutable_variable_parameter(std::string const &variable_name)
{
    if (variable_name == this->get_name() + ".position.base")
    {
        return position_variable;
    }
    else if (variable_name == this->get_name() + ".linear_velocity.base")
    {
        return linear_velocity_variable;
    }
    else if (variable_name == this->get_name() + ".aligned_linear_velocity.base")
    {
        return aligned_linear_velocity_variable;
    }
    else if (variable_name == this->get_name() + ".linear_acceleration.base")
    {
        return linear_acceleration_variable;
    }
    else if (variable_name == this->get_name() + ".aligned_linear_acceleration.indirect_actuation")
    {
        return aligned_linear_acceleration_variable;
    }
    else if (variable_name == this->get_name() + ".linear_acceleration.external")
    {
        return external_linear_acceleration_variable;
    }
    else if (variable_name == this->get_name() + ".rotation.base")
    {
        return rotation_variable;
    }
    else if (variable_name == this->get_name() + ".steer.indirect_actuation")
    {
        return steer_variable;
    }
    else if (variable_name == this->get_name() + ".angular_velocity.base")
    {
        return angular_velocity_variable;
    }
    else if (variable_name == this->get_name() + ".ttc.base")
    {
        return ttc_variable;
    }
    else if (variable_name == this->get_name() + ".cumilative_collision_time.base")
    {
        return cumilative_collision_time_variable;
    }
    else
    {
        return nullptr;
    }
}

structures::IArray<IValuelessEvent*>* HighDDrivingAgent::get_mutable_events()
{
    structures::stl::STLConcatArray<IValuelessEvent*> *events =
            new structures::stl::STLConcatArray<IValuelessEvent*>(11);

    events->get_array(0) = position_variable->get_mutable_valueless_events();
    events->get_array(1) = linear_velocity_variable->get_mutable_valueless_events();
    events->get_array(2) = aligned_linear_velocity_variable->get_mutable_valueless_events();
    events->get_array(3) = linear_acceleration_variable->get_mutable_valueless_events();
    events->get_array(4) = aligned_linear_acceleration_variable->get_mutable_valueless_events();
    events->get_array(5) = external_linear_acceleration_variable->get_mutable_valueless_events();
    events->get_array(6) = rotation_variable->get_mutable_valueless_events();
    events->get_array(7) = steer_variable->get_mutable_valueless_events();
    events->get_array(8) = angular_velocity_variable->get_mutable_valueless_events();
    events->get_array(9) = ttc_variable->get_mutable_valueless_events();
    events->get_array(10) = cumilative_collision_time_variable->get_mutable_valueless_events();

    return events;
}

IConstant<uint32_t>* HighDDrivingAgent::get_mutable_id_constant()
{
    return id_constant;
}

IConstant<bool>* HighDDrivingAgent::get_mutable_ego_constant()
{
    return ego_constant;
}

IConstant<FP_DATA_TYPE>* HighDDrivingAgent::get_mutable_bb_length_constant()
{
    return bb_length_constant;
}

IConstant<FP_DATA_TYPE>* HighDDrivingAgent::get_mutable_bb_width_constant()
{
    return bb_width_constant;
}

IConstant<DrivingAgentClass>* HighDDrivingAgent::get_mutable_driving_agent_class_constant()
{
    return driving_agent_class_constant;
}

IVariable<geometry::Vec>* HighDDrivingAgent::get_mutable_position_variable()
{
    return position_variable;
}

IVariable<geometry::Vec>* HighDDrivingAgent::get_mutable_linear_velocity_variable()
{
    return linear_velocity_variable;
}

IVariable<FP_DATA_TYPE>* HighDDrivingAgent::get_mutable_aligned_linear_velocity_variable()
{
    return aligned_linear_velocity_variable;
}

IVariable<geometry::Vec>* HighDDrivingAgent::get_mutable_linear_acceleration_variable()
{
    return linear_acceleration_variable;
}

IVariable<FP_DATA_TYPE>* HighDDrivingAgent::get_mutable_aligned_linear_acceleration_variable()
{
    return aligned_linear_acceleration_variable;
}

IVariable<geometry::Vec>* HighDDrivingAgent::get_mutable_external_linear_acceleration_variable()
{
    return external_linear_acceleration_variable;
}

IVariable<FP_DATA_TYPE>* HighDDrivingAgent::get_mutable_rotation_variable()
{
    return rotation_variable;
}

IVariable<FP_DATA_TYPE>* HighDDrivingAgent::get_mutable_steer_variable()
{
    return steer_variable;
}

IVariable<FP_DATA_TYPE>* HighDDrivingAgent::get_mutable_angular_velocity_variable()
{
    return angular_velocity_variable;
}

IVariable<temporal::Duration>* HighDDrivingAgent::get_mutable_ttc_variable()
{
    return ttc_variable;
}

IVariable<temporal::Duration>* HighDDrivingAgent::get_mutable_cumilative_collision_time_variable()
{
    return cumilative_collision_time_variable;
}

}
}
}
}
