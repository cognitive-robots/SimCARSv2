
#include <ori/simcars/agent/driving_agent_abstract.hpp>
#include <ori/simcars/agent/basic_constant.hpp>
#include <ori/simcars/agent/basic_driving_agent_state.hpp>

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

IState* ADrivingAgent::get_state(temporal::Time time, bool throw_on_out_of_range) const
{
    return this->get_driving_agent_state(time, throw_on_out_of_range);
}

IDrivingAgentState* ADrivingAgent::get_driving_agent_state(temporal::Time time, bool throw_on_out_of_range) const
{
    IDrivingAgentState *driving_agent_state(new BasicDrivingAgentState(this->get_name()));

    driving_agent_state->set_id_constant(this->get_id_constant());
    driving_agent_state->set_ego_constant(this->get_ego_constant());
    driving_agent_state->set_bb_length_constant(this->get_bb_length_constant());
    driving_agent_state->set_bb_width_constant(this->get_bb_width_constant());
    driving_agent_state->set_driving_agent_class_constant(this->get_driving_agent_class_constant());

    try
    {
        //if (driving_agent_state->get_id_constant()->get_value() == 1310)
        //{
            //std::cerr << "ADrivingAgent: " << this->get_position_variable()->get_min_temporal_limit().time_since_epoch().count() << std::endl;
        //}
        driving_agent_state->set_position_variable(
                        new BasicConstant(
                            this->get_name(),
                            "position.base",
                            this->get_position_variable()->get_value(time)));
        driving_agent_state->set_linear_velocity_variable(
                        new BasicConstant(
                            this->get_name(),
                            "linear_velocity.base",
                            this->get_linear_velocity_variable()->get_value(time)));
        driving_agent_state->set_aligned_linear_velocity_variable(
                        new BasicConstant(
                            this->get_name(),
                            "aligned_linear_velocity.base",
                            this->get_aligned_linear_velocity_variable()->get_value(time)));
        driving_agent_state->set_linear_acceleration_variable(
                        new BasicConstant(
                            this->get_name(),
                            "linear_acceleration.base",
                            this->get_linear_acceleration_variable()->get_value(time)));
        driving_agent_state->set_aligned_linear_acceleration_variable(
                        new BasicConstant(
                            this->get_name(),
                            "aligned_linear_acceleration.indirect_actuation",
                            this->get_aligned_linear_acceleration_variable()->get_value(time)));
        driving_agent_state->set_external_linear_acceleration_variable(
                        new BasicConstant(
                            this->get_name(),
                            "linear_acceleration.external",
                            this->get_external_linear_acceleration_variable()->get_value(time)));
        driving_agent_state->set_rotation_variable(
                        new BasicConstant(
                            this->get_name(),
                            "rotation.base",
                            this->get_rotation_variable()->get_value(time)));
        driving_agent_state->set_steer_variable(
                        new BasicConstant(
                            this->get_name(),
                            "steer.indirect_actuation",
                            this->get_steer_variable()->get_value(time)));
        driving_agent_state->set_angular_velocity_variable(
                        new BasicConstant(
                            this->get_name(),
                            "angular_velocity.base",
                            this->get_angular_velocity_variable()->get_value(time)));
    }
    catch (std::out_of_range const &e)
    {
        if (throw_on_out_of_range)
        {
            throw e;
        }
    }

    return driving_agent_state;
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

}
}
}
