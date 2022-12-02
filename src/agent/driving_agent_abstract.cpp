
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/agent/driving_agent_abstract.hpp>
#include <ori/simcars/agent/view_read_only_driving_agent_state.hpp>
#include <ori/simcars/agent/view_driving_agent_state.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace agent
{

IEntity* ADrivingAgent::entity_deep_copy() const
{
    return this->driving_agent_deep_copy();
}

bool ADrivingAgent::is_state_available(temporal::Time time) const
{
    return time >= this->get_min_temporal_limit() && time <= this->get_max_temporal_limit();
}

IReadOnlyEntityState const* ADrivingAgent::get_state(temporal::Time time) const
{
    return this->get_driving_agent_state(time);
}

IConstant<uint32_t> const* ADrivingAgent::get_id_constant() const
{
    IValuelessConstant const *id_valueless_constant =
            this->get_constant_parameter(this->get_name() + ".id");
    return dynamic_cast<IConstant<uint32_t> const*>(id_valueless_constant);
}

IConstant<bool> const* ADrivingAgent::get_ego_constant() const
{
    IValuelessConstant const *ego_valueless_constant =
            this->get_constant_parameter(this->get_name() + ".ego");
    return dynamic_cast<IConstant<bool> const*>(ego_valueless_constant);
}

IConstant<FP_DATA_TYPE> const* ADrivingAgent::get_bb_length_constant() const
{
    IValuelessConstant const *bb_length_valueless_constant =
            this->get_constant_parameter(this->get_name() + ".bb_length");
    return dynamic_cast<IConstant<FP_DATA_TYPE> const*>(bb_length_valueless_constant);
}

IConstant<FP_DATA_TYPE> const* ADrivingAgent::get_bb_width_constant() const
{
    IValuelessConstant const *bb_width_valueless_constant =
            this->get_constant_parameter(this->get_name() + ".bb_width");
    return dynamic_cast<IConstant<FP_DATA_TYPE> const*>(bb_width_valueless_constant);
}

IConstant<DrivingAgentClass> const* ADrivingAgent::get_driving_agent_class_constant() const
{
    IValuelessConstant const *driving_agent_class_valueless_constant =
            this->get_constant_parameter(this->get_name() + ".driving_agent_class");
    return dynamic_cast<IConstant<DrivingAgentClass> const*>(driving_agent_class_valueless_constant);
}

IVariable<geometry::Vec> const* ADrivingAgent::get_position_variable() const
{
    IValuelessVariable const *position_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".position.base");
    return dynamic_cast<IVariable<geometry::Vec> const*>(position_valueless_variable);
}

IVariable<geometry::Vec> const* ADrivingAgent::get_linear_velocity_variable() const
{
    IValuelessVariable const *linear_velocity_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".linear_velocity.base");
    return dynamic_cast<IVariable<geometry::Vec> const*>(linear_velocity_valueless_variable);
}

IVariable<FP_DATA_TYPE> const* ADrivingAgent::get_aligned_linear_velocity_variable() const
{
    IValuelessVariable const *aligned_linear_velocity_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".aligned_linear_velocity.base");
    return dynamic_cast<IVariable<FP_DATA_TYPE> const*>(aligned_linear_velocity_valueless_variable);
}

IVariable<geometry::Vec> const* ADrivingAgent::get_linear_acceleration_variable() const
{
    IValuelessVariable const *linear_acceleration_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".linear_acceleration.base");
    return dynamic_cast<IVariable<geometry::Vec> const*>(linear_acceleration_valueless_variable);
}

IVariable<FP_DATA_TYPE> const* ADrivingAgent::get_aligned_linear_acceleration_variable() const
{
    IValuelessVariable const *aligned_linear_acceleration_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".aligned_linear_acceleration.indirect_actuation");
    return dynamic_cast<IVariable<FP_DATA_TYPE> const*>(aligned_linear_acceleration_valueless_variable);
}

IVariable<geometry::Vec> const* ADrivingAgent::get_external_linear_acceleration_variable() const
{
    IValuelessVariable const *linear_acceleration_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".linear_acceleration.external");
    return dynamic_cast<IVariable<geometry::Vec> const*>(linear_acceleration_valueless_variable);
}

IVariable<FP_DATA_TYPE> const* ADrivingAgent::get_rotation_variable() const
{
    IValuelessVariable const *rotation_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".rotation.base");
    return dynamic_cast<IVariable<FP_DATA_TYPE> const*>(rotation_valueless_variable);
}

IVariable<FP_DATA_TYPE> const* ADrivingAgent::get_steer_variable() const
{
    IValuelessVariable const *steer_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".steer.indirect_actuation");
    return dynamic_cast<IVariable<FP_DATA_TYPE> const*>(steer_valueless_variable);
}

IVariable<FP_DATA_TYPE> const* ADrivingAgent::get_angular_velocity_variable() const
{
    IValuelessVariable const *angular_velocity_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".angular_velocity.base");
    return dynamic_cast<IVariable<FP_DATA_TYPE> const*>(angular_velocity_valueless_variable);
}

IReadOnlyDrivingAgentState const* ADrivingAgent::get_driving_agent_state(temporal::Time time) const
{
    return new ViewReadOnlyDrivingAgentState(this, time);
}

IEntityState* ADrivingAgent::get_mutable_state(temporal::Time time)
{
    return this->get_mutable_driving_agent_state(time);
}

IConstant<uint32_t>* ADrivingAgent::get_mutable_id_constant()
{
    IValuelessConstant *id_valueless_constant =
            this->get_mutable_constant_parameter(this->get_name() + ".id");
    return dynamic_cast<IConstant<uint32_t>*>(id_valueless_constant);
}

IConstant<bool>* ADrivingAgent::get_mutable_ego_constant()
{
    IValuelessConstant *ego_valueless_constant =
            this->get_mutable_constant_parameter(this->get_name() + ".ego");
    return dynamic_cast<IConstant<bool>*>(ego_valueless_constant);
}

IConstant<FP_DATA_TYPE>* ADrivingAgent::get_mutable_bb_length_constant()
{
    IValuelessConstant *bb_length_valueless_constant =
            this->get_mutable_constant_parameter(this->get_name() + ".bb_length");
    return dynamic_cast<IConstant<FP_DATA_TYPE>*>(bb_length_valueless_constant);
}

IConstant<FP_DATA_TYPE>* ADrivingAgent::get_mutable_bb_width_constant()
{
    IValuelessConstant *bb_width_valueless_constant =
            this->get_mutable_constant_parameter(this->get_name() + ".bb_width");
    return dynamic_cast<IConstant<FP_DATA_TYPE>*>(bb_width_valueless_constant);
}

IConstant<DrivingAgentClass>* ADrivingAgent::get_mutable_driving_agent_class_constant()
{
    IValuelessConstant *driving_agent_class_valueless_constant =
            this->get_mutable_constant_parameter(this->get_name() + ".driving_agent_class");
    return dynamic_cast<IConstant<DrivingAgentClass>*>(driving_agent_class_valueless_constant);
}

IVariable<geometry::Vec>* ADrivingAgent::get_mutable_position_variable()
{
    IValuelessVariable *position_valueless_variable =
            this->get_mutable_variable_parameter(this->get_name() + ".position.base");
    return dynamic_cast<IVariable<geometry::Vec>*>(position_valueless_variable);
}

IVariable<geometry::Vec>* ADrivingAgent::get_mutable_linear_velocity_variable()
{
    IValuelessVariable *linear_velocity_valueless_variable =
            this->get_mutable_variable_parameter(this->get_name() + ".linear_velocity.base");
    return dynamic_cast<IVariable<geometry::Vec>*>(linear_velocity_valueless_variable);
}

IVariable<FP_DATA_TYPE>* ADrivingAgent::get_mutable_aligned_linear_velocity_variable()
{
    IValuelessVariable *aligned_linear_velocity_valueless_variable =
            this->get_mutable_variable_parameter(this->get_name() + ".aligned_linear_velocity.base");
    return dynamic_cast<IVariable<FP_DATA_TYPE>*>(aligned_linear_velocity_valueless_variable);
}

IVariable<geometry::Vec>* ADrivingAgent::get_mutable_linear_acceleration_variable()
{
    IValuelessVariable *linear_acceleration_valueless_variable =
            this->get_mutable_variable_parameter(this->get_name() + ".linear_acceleration.base");
    return dynamic_cast<IVariable<geometry::Vec>*>(linear_acceleration_valueless_variable);
}

IVariable<FP_DATA_TYPE>* ADrivingAgent::get_mutable_aligned_linear_acceleration_variable()
{
    IValuelessVariable *aligned_linear_acceleration_valueless_variable =
            this->get_mutable_variable_parameter(this->get_name() + ".aligned_linear_acceleration.indirect_actuation");
    return dynamic_cast<IVariable<FP_DATA_TYPE>*>(aligned_linear_acceleration_valueless_variable);
}

IVariable<geometry::Vec>* ADrivingAgent::get_mutable_external_linear_acceleration_variable()
{
    IValuelessVariable *linear_acceleration_valueless_variable =
            this->get_mutable_variable_parameter(this->get_name() + ".linear_acceleration.external");
    return dynamic_cast<IVariable<geometry::Vec>*>(linear_acceleration_valueless_variable);
}

IVariable<FP_DATA_TYPE>* ADrivingAgent::get_mutable_rotation_variable()
{
    IValuelessVariable *rotation_valueless_variable =
            this->get_mutable_variable_parameter(this->get_name() + ".rotation.base");
    return dynamic_cast<IVariable<FP_DATA_TYPE>*>(rotation_valueless_variable);
}

IVariable<FP_DATA_TYPE>* ADrivingAgent::get_mutable_steer_variable()
{
    IValuelessVariable *steer_valueless_variable =
            this->get_mutable_variable_parameter(this->get_name() + ".steer.indirect_actuation");
    return dynamic_cast<IVariable<FP_DATA_TYPE>*>(steer_valueless_variable);
}

IVariable<FP_DATA_TYPE>* ADrivingAgent::get_mutable_angular_velocity_variable()
{
    IValuelessVariable *angular_velocity_valueless_variable =
            this->get_mutable_variable_parameter(this->get_name() + ".angular_velocity.base");
    return dynamic_cast<IVariable<FP_DATA_TYPE>*>(angular_velocity_valueless_variable);
}

IDrivingAgentState* ADrivingAgent::get_mutable_driving_agent_state(temporal::Time time)
{
    return new ViewDrivingAgentState(this, time);
}

}
}
}
