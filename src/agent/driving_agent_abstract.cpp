
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

std::shared_ptr<IEntity> ADrivingAgent::entity_deep_copy() const
{
    return this->driving_agent_deep_copy();
}

std::shared_ptr<IState> ADrivingAgent::get_state(temporal::Time time, bool throw_on_out_of_range) const
{
    return this->get_driving_agent_state(time, throw_on_out_of_range);
}

std::shared_ptr<IDrivingAgentState> ADrivingAgent::get_driving_agent_state(temporal::Time time, bool throw_on_out_of_range) const
{
    std::shared_ptr<IDrivingAgentState> driving_agent_state(new BasicDrivingAgentState(this->get_name()));

    driving_agent_state->set_id_constant(this->get_id_constant());
    driving_agent_state->set_ego_constant(this->get_ego_constant());
    driving_agent_state->set_bb_length_constant(this->get_bb_length_constant());
    driving_agent_state->set_bb_width_constant(this->get_bb_width_constant());
    driving_agent_state->set_driving_agent_class_constant(this->get_driving_agent_class_constant());

    try
    {
        if (driving_agent_state->get_id_constant()->get_value() == 1310)
        {
            //std::cerr << "ADrivingAgent: " << this->get_position_variable()->get_min_temporal_limit().time_since_epoch().count() << std::endl;
        }
        driving_agent_state->set_position_variable(
                    std::shared_ptr<const IConstant<geometry::Vec>>(
                        new BasicConstant(
                            this->get_name(),
                            "position.base",
                            this->get_position_variable()->get_value(time))));
        driving_agent_state->set_linear_velocity_variable(
                    std::shared_ptr<const IConstant<geometry::Vec>>(
                        new BasicConstant(
                            this->get_name(),
                            "linear_velocity.base",
                            this->get_linear_velocity_variable()->get_value(time))));
        driving_agent_state->set_aligned_linear_velocity_variable(
                    std::shared_ptr<const IConstant<FP_DATA_TYPE>>(
                        new BasicConstant(
                            this->get_name(),
                            "aligned_linear_velocity.base",
                            this->get_aligned_linear_velocity_variable()->get_value(time))));
        driving_agent_state->set_linear_acceleration_variable(
                    std::shared_ptr<const IConstant<geometry::Vec>>(
                        new BasicConstant(
                            this->get_name(),
                            "linear_acceleration.base",
                            this->get_linear_acceleration_variable()->get_value(time))));
        driving_agent_state->set_aligned_linear_acceleration_variable(
                    std::shared_ptr<const IConstant<FP_DATA_TYPE>>(
                        new BasicConstant(
                            this->get_name(),
                            "aligned_linear_acceleration.indirect_actuation",
                            this->get_aligned_linear_acceleration_variable()->get_value(time))));
        driving_agent_state->set_external_linear_acceleration_variable(
                    std::shared_ptr<const IConstant<geometry::Vec>>(
                        new BasicConstant(
                            this->get_name(),
                            "linear_acceleration.external",
                            this->get_external_linear_acceleration_variable()->get_value(time))));
        driving_agent_state->set_rotation_variable(
                    std::shared_ptr<const IConstant<FP_DATA_TYPE>>(
                        new BasicConstant(
                            this->get_name(),
                            "rotation.base",
                            this->get_rotation_variable()->get_value(time))));
        driving_agent_state->set_steer_variable(
                    std::shared_ptr<const IConstant<FP_DATA_TYPE>>(
                        new BasicConstant(
                            this->get_name(),
                            "steer.indirect_actuation",
                            this->get_steer_variable()->get_value(time))));
        driving_agent_state->set_angular_velocity_variable(
                    std::shared_ptr<const IConstant<FP_DATA_TYPE>>(
                        new BasicConstant(
                            this->get_name(),
                            "angular_velocity.base",
                            this->get_angular_velocity_variable()->get_value(time))));
    }
    catch (std::out_of_range e)
    {
        if (throw_on_out_of_range)
        {
            throw e;
        }
    }

    return driving_agent_state;
}

std::shared_ptr<const IConstant<uint32_t>> ADrivingAgent::get_id_constant() const
{
    std::shared_ptr<const IValuelessConstant> id_valueless_constant =
            this->get_constant_parameter(this->get_name() + ".id");
    return std::dynamic_pointer_cast<const IConstant<uint32_t>>(id_valueless_constant);
}

std::shared_ptr<const IConstant<bool>> ADrivingAgent::get_ego_constant() const
{
    std::shared_ptr<const IValuelessConstant> ego_valueless_constant =
            this->get_constant_parameter(this->get_name() + ".ego");
    return std::dynamic_pointer_cast<const IConstant<bool>>(ego_valueless_constant);
}

std::shared_ptr<const IConstant<FP_DATA_TYPE>> ADrivingAgent::get_bb_length_constant() const
{
    std::shared_ptr<const IValuelessConstant> bb_length_valueless_constant =
            this->get_constant_parameter(this->get_name() + ".bb_length");
    return std::dynamic_pointer_cast<const IConstant<FP_DATA_TYPE>>(bb_length_valueless_constant);
}

std::shared_ptr<const IConstant<FP_DATA_TYPE>> ADrivingAgent::get_bb_width_constant() const
{
    std::shared_ptr<const IValuelessConstant> bb_width_valueless_constant =
            this->get_constant_parameter(this->get_name() + ".bb_width");
    return std::dynamic_pointer_cast<const IConstant<FP_DATA_TYPE>>(bb_width_valueless_constant);
}

std::shared_ptr<const IConstant<DrivingAgentClass>> ADrivingAgent::get_driving_agent_class_constant() const
{
    std::shared_ptr<const IValuelessConstant> driving_agent_class_valueless_constant =
            this->get_constant_parameter(this->get_name() + ".driving_agent_class");
    return std::dynamic_pointer_cast<const IConstant<DrivingAgentClass>>(driving_agent_class_valueless_constant);
}

std::shared_ptr<const IVariable<geometry::Vec>> ADrivingAgent::get_position_variable() const
{
    std::shared_ptr<const IValuelessVariable> position_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".position.base");
    return std::dynamic_pointer_cast<const IVariable<geometry::Vec>>(position_valueless_variable);
}

std::shared_ptr<const IVariable<geometry::Vec>> ADrivingAgent::get_linear_velocity_variable() const
{
    std::shared_ptr<const IValuelessVariable> linear_velocity_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".linear_velocity.base");
    return std::dynamic_pointer_cast<const IVariable<geometry::Vec>>(linear_velocity_valueless_variable);
}

std::shared_ptr<const IVariable<FP_DATA_TYPE>> ADrivingAgent::get_aligned_linear_velocity_variable() const
{
    std::shared_ptr<const IValuelessVariable> aligned_linear_velocity_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".aligned_linear_velocity.base");
    return std::dynamic_pointer_cast<const IVariable<FP_DATA_TYPE>>(aligned_linear_velocity_valueless_variable);
}

std::shared_ptr<const IVariable<geometry::Vec>> ADrivingAgent::get_linear_acceleration_variable() const
{
    std::shared_ptr<const IValuelessVariable> linear_acceleration_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".linear_acceleration.base");
    return std::dynamic_pointer_cast<const IVariable<geometry::Vec>>(linear_acceleration_valueless_variable);
}

std::shared_ptr<const IVariable<FP_DATA_TYPE>> ADrivingAgent::get_aligned_linear_acceleration_variable() const
{
    std::shared_ptr<const IValuelessVariable> aligned_linear_acceleration_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".aligned_linear_acceleration.indirect_actuation");
    return std::dynamic_pointer_cast<const IVariable<FP_DATA_TYPE>>(aligned_linear_acceleration_valueless_variable);
}

std::shared_ptr<const IVariable<geometry::Vec>> ADrivingAgent::get_external_linear_acceleration_variable() const
{
    std::shared_ptr<const IValuelessVariable> linear_acceleration_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".linear_acceleration.external");
    return std::dynamic_pointer_cast<const IVariable<geometry::Vec>>(linear_acceleration_valueless_variable);
}

std::shared_ptr<const IVariable<FP_DATA_TYPE>> ADrivingAgent::get_rotation_variable() const
{
    std::shared_ptr<const IValuelessVariable> rotation_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".rotation.base");
    return std::dynamic_pointer_cast<const IVariable<FP_DATA_TYPE>>(rotation_valueless_variable);
}

std::shared_ptr<const IVariable<FP_DATA_TYPE>> ADrivingAgent::get_steer_variable() const
{
    std::shared_ptr<const IValuelessVariable> steer_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".steer.indirect_actuation");
    return std::dynamic_pointer_cast<const IVariable<FP_DATA_TYPE>>(steer_valueless_variable);
}

std::shared_ptr<const IVariable<FP_DATA_TYPE>> ADrivingAgent::get_angular_velocity_variable() const
{
    std::shared_ptr<const IValuelessVariable> angular_velocity_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".angular_velocity.base");
    return std::dynamic_pointer_cast<const IVariable<FP_DATA_TYPE>>(angular_velocity_valueless_variable);
}

}
}
}
