
#include <ori/simcars/agent/driving_agent_state_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

IConstant<uint32_t> const* AReadOnlyDrivingAgentState::get_id_constant() const
{
    IValuelessConstant const *id_valueless_constant =
            this->get_parameter_value(this->get_name() + ".id");
    return dynamic_cast<IConstant<uint32_t> const*>(id_valueless_constant);
}

IConstant<bool> const* AReadOnlyDrivingAgentState::get_ego_constant() const
{
    IValuelessConstant const *ego_valueless_constant =
            this->get_parameter_value(this->get_name() + ".ego");
    return dynamic_cast<IConstant<bool> const*>(ego_valueless_constant);
}

IConstant<FP_DATA_TYPE> const* AReadOnlyDrivingAgentState::get_bb_length_constant() const
{
    IValuelessConstant const *bb_length_valueless_constant =
            this->get_parameter_value(this->get_name() + ".bb_length");
    return dynamic_cast<IConstant<FP_DATA_TYPE> const*>(bb_length_valueless_constant);
}

IConstant<FP_DATA_TYPE> const* AReadOnlyDrivingAgentState::get_bb_width_constant() const
{
    IValuelessConstant const *bb_width_valueless_constant =
            this->get_parameter_value(this->get_name() + ".bb_width");
    return dynamic_cast<IConstant<FP_DATA_TYPE> const*>(bb_width_valueless_constant);
}

IConstant<DrivingAgentClass> const* AReadOnlyDrivingAgentState::get_driving_agent_class_constant() const
{
    IValuelessConstant const *driving_agent_class_valueless_constant =
            this->get_parameter_value(this->get_name() + ".driving_agent_class");
    return dynamic_cast<IConstant<DrivingAgentClass> const*>(driving_agent_class_valueless_constant);
}

IConstant<geometry::Vec> const* AReadOnlyDrivingAgentState::get_position_variable() const
{
    IValuelessConstant const *position_valueless_variable =
            this->get_parameter_value(this->get_name() + ".position.base");
    return dynamic_cast<IConstant<geometry::Vec> const*>(position_valueless_variable);
}

IConstant<geometry::Vec> const* AReadOnlyDrivingAgentState::get_linear_velocity_variable() const
{
    IValuelessConstant const *linear_velocity_valueless_variable =
            this->get_parameter_value(this->get_name() + ".linear_velocity.base");
    return dynamic_cast<IConstant<geometry::Vec> const*>(linear_velocity_valueless_variable);
}

IConstant<FP_DATA_TYPE> const* AReadOnlyDrivingAgentState::get_aligned_linear_velocity_variable() const
{
    IValuelessConstant const *aligned_linear_velocity_valueless_variable =
            this->get_parameter_value(this->get_name() + ".aligned_linear_velocity.base");
    return dynamic_cast<IConstant<FP_DATA_TYPE> const*>(aligned_linear_velocity_valueless_variable);
}

IConstant<geometry::Vec> const* AReadOnlyDrivingAgentState::get_linear_acceleration_variable() const
{
    IValuelessConstant const *linear_acceleration_valueless_variable =
            this->get_parameter_value(this->get_name() + ".linear_acceleration.base");
    return dynamic_cast<IConstant<geometry::Vec> const*>(linear_acceleration_valueless_variable);
}

IConstant<FP_DATA_TYPE> const* AReadOnlyDrivingAgentState::get_aligned_linear_acceleration_variable() const
{
    IValuelessConstant const *aligned_linear_acceleration_valueless_variable =
            this->get_parameter_value(this->get_name() + ".aligned_linear_acceleration.indirect_actuation");
    return dynamic_cast<IConstant<FP_DATA_TYPE> const*>(aligned_linear_acceleration_valueless_variable);
}

IConstant<geometry::Vec> const* AReadOnlyDrivingAgentState::get_external_linear_acceleration_variable() const
{
    IValuelessConstant const *linear_acceleration_valueless_variable =
            this->get_parameter_value(this->get_name() + ".linear_acceleration.external");
    return dynamic_cast<IConstant<geometry::Vec> const*>(linear_acceleration_valueless_variable);
}

IConstant<FP_DATA_TYPE> const* AReadOnlyDrivingAgentState::get_rotation_variable() const
{
    IValuelessConstant const *rotation_valueless_variable =
            this->get_parameter_value(this->get_name() + ".rotation.base");
    return dynamic_cast<IConstant<FP_DATA_TYPE> const*>(rotation_valueless_variable);
}

IConstant<FP_DATA_TYPE> const* AReadOnlyDrivingAgentState::get_steer_variable() const
{
    IValuelessConstant const *steer_valueless_variable =
            this->get_parameter_value(this->get_name() + ".steer.indirect_actuation");
    return dynamic_cast<IConstant<FP_DATA_TYPE> const*>(steer_valueless_variable);
}

IConstant<FP_DATA_TYPE> const* AReadOnlyDrivingAgentState::get_angular_velocity_variable() const
{
    IValuelessConstant const *angular_velocity_valueless_variable =
            this->get_parameter_value(this->get_name() + ".angular_velocity.base");
    return dynamic_cast<IConstant<FP_DATA_TYPE> const*>(angular_velocity_valueless_variable);
}

IConstant<temporal::Duration> const* AReadOnlyDrivingAgentState::get_ttc_variable() const
{
    IValuelessConstant const *ttc_valueless_variable =
            this->get_parameter_value(this->get_name() + ".ttc.base");
    return dynamic_cast<IConstant<temporal::Duration> const*>(ttc_valueless_variable);
}

IConstant<temporal::Duration> const* AReadOnlyDrivingAgentState::get_cumilative_collision_time_variable() const
{
    IValuelessConstant const *cumilative_collision_time_valueless_variable =
            this->get_parameter_value(this->get_name() + ".cumilative_collision_time.base");
    return dynamic_cast<IConstant<temporal::Duration> const*>(cumilative_collision_time_valueless_variable);
}

}
}
}
