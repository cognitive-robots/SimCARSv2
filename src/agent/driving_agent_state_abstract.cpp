
#include <ori/simcars/agent/driving_agent_state_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

std::shared_ptr<const IConstant<uint32_t>> ADrivingAgentState::get_id_constant() const
{
    std::shared_ptr<const IValuelessConstant> id_valueless_constant =
            this->get_parameter_value(this->get_driving_agent_name() + ".id");
    return std::dynamic_pointer_cast<const IConstant<uint32_t>>(id_valueless_constant);
}

std::shared_ptr<const IConstant<bool>> ADrivingAgentState::get_ego_constant() const
{
    std::shared_ptr<const IValuelessConstant> ego_valueless_constant =
            this->get_parameter_value(this->get_driving_agent_name() + ".ego");
    return std::dynamic_pointer_cast<const IConstant<bool>>(ego_valueless_constant);
}

std::shared_ptr<const IConstant<FP_DATA_TYPE>> ADrivingAgentState::get_bb_length_constant() const
{
    std::shared_ptr<const IValuelessConstant> bb_length_valueless_constant =
            this->get_parameter_value(this->get_driving_agent_name() + ".bb_length");
    return std::dynamic_pointer_cast<const IConstant<FP_DATA_TYPE>>(bb_length_valueless_constant);
}

std::shared_ptr<const IConstant<FP_DATA_TYPE>> ADrivingAgentState::get_bb_width_constant() const
{
    std::shared_ptr<const IValuelessConstant> bb_width_valueless_constant =
            this->get_parameter_value(this->get_driving_agent_name() + ".bb_width");
    return std::dynamic_pointer_cast<const IConstant<FP_DATA_TYPE>>(bb_width_valueless_constant);
}

std::shared_ptr<const IConstant<DrivingAgentClass>> ADrivingAgentState::get_driving_agent_class_constant() const
{
    std::shared_ptr<const IValuelessConstant> driving_agent_class_valueless_constant =
            this->get_parameter_value(this->get_driving_agent_name() + ".driving_agent_class");
    return std::dynamic_pointer_cast<const IConstant<DrivingAgentClass>>(driving_agent_class_valueless_constant);
}

std::shared_ptr<const IConstant<geometry::Vec>> ADrivingAgentState::get_position_variable() const
{
    std::shared_ptr<const IValuelessConstant> position_valueless_variable =
            this->get_parameter_value(this->get_driving_agent_name() + ".position.base");
    return std::dynamic_pointer_cast<const IConstant<geometry::Vec>>(position_valueless_variable);
}

std::shared_ptr<const IConstant<geometry::Vec>> ADrivingAgentState::get_linear_velocity_variable() const
{
    std::shared_ptr<const IValuelessConstant> linear_velocity_valueless_variable =
            this->get_parameter_value(this->get_driving_agent_name() + ".linear_velocity.base");
    return std::dynamic_pointer_cast<const IConstant<geometry::Vec>>(linear_velocity_valueless_variable);
}

std::shared_ptr<const IConstant<FP_DATA_TYPE>> ADrivingAgentState::get_aligned_linear_velocity_variable() const
{
    std::shared_ptr<const IValuelessConstant> aligned_linear_velocity_valueless_variable =
            this->get_parameter_value(this->get_driving_agent_name() + ".aligned_linear_velocity.base");
    return std::dynamic_pointer_cast<const IConstant<FP_DATA_TYPE>>(aligned_linear_velocity_valueless_variable);
}

std::shared_ptr<const IConstant<geometry::Vec>> ADrivingAgentState::get_linear_acceleration_variable() const
{
    std::shared_ptr<const IValuelessConstant> linear_acceleration_valueless_variable =
            this->get_parameter_value(this->get_driving_agent_name() + ".linear_acceleration.base");
    return std::dynamic_pointer_cast<const IConstant<geometry::Vec>>(linear_acceleration_valueless_variable);
}

std::shared_ptr<const IConstant<FP_DATA_TYPE>> ADrivingAgentState::get_aligned_linear_acceleration_variable() const
{
    std::shared_ptr<const IValuelessConstant> aligned_linear_acceleration_valueless_variable =
            this->get_parameter_value(this->get_driving_agent_name() + ".aligned_linear_acceleration.indirect_actuation");
    return std::dynamic_pointer_cast<const IConstant<FP_DATA_TYPE>>(aligned_linear_acceleration_valueless_variable);
}

std::shared_ptr<const IConstant<geometry::Vec>> ADrivingAgentState::get_external_linear_acceleration_variable() const
{
    std::shared_ptr<const IValuelessConstant> external_linear_acceleration_valueless_variable =
            this->get_parameter_value(this->get_driving_agent_name() + ".linear_acceleration.external");
    return std::dynamic_pointer_cast<const IConstant<geometry::Vec>>(external_linear_acceleration_valueless_variable);
}

std::shared_ptr<const IConstant<FP_DATA_TYPE>> ADrivingAgentState::get_rotation_variable() const
{
    std::shared_ptr<const IValuelessConstant> rotation_valueless_variable =
            this->get_parameter_value(this->get_driving_agent_name() + ".rotation.base");
    return std::dynamic_pointer_cast<const IConstant<FP_DATA_TYPE>>(rotation_valueless_variable);
}

std::shared_ptr<const IConstant<FP_DATA_TYPE>> ADrivingAgentState::get_steer_variable() const
{
    std::shared_ptr<const IValuelessConstant> steer_valueless_variable =
            this->get_parameter_value(this->get_driving_agent_name() + ".steer.indirect_actuation");
    return std::dynamic_pointer_cast<const IConstant<FP_DATA_TYPE>>(steer_valueless_variable);
}

std::shared_ptr<const IConstant<FP_DATA_TYPE>> ADrivingAgentState::get_angular_velocity_variable() const
{
    std::shared_ptr<const IValuelessConstant> angular_velocity_valueless_variable =
            this->get_parameter_value(this->get_driving_agent_name() + ".angular_velocity.base");
    return std::dynamic_pointer_cast<const IConstant<FP_DATA_TYPE>>(angular_velocity_valueless_variable);
}

}
}
}
