
#include <ori/simcars/agent/driving_agent_abstract.hpp>

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

std::shared_ptr<const IState> ADrivingAgent::get_state(temporal::Time time) const
{
    return this->get_driving_agent_state(time);
}

std::shared_ptr<const IConstant<uint32_t>> ADrivingAgent::get_id_constant() const
{
    std::shared_ptr<const IValuelessConstant> id_valueless_constant =
            this->get_constant_parameter(this->get_name() + ".id");
    return std::static_pointer_cast<const IConstant<uint32_t>>(id_valueless_constant);
}

std::shared_ptr<const IConstant<bool>> ADrivingAgent::get_ego_constant() const
{
    std::shared_ptr<const IValuelessConstant> ego_valueless_constant =
            this->get_constant_parameter(this->get_name() + ".ego");
    return std::static_pointer_cast<const IConstant<bool>>(ego_valueless_constant);
}

std::shared_ptr<const IConstant<FP_DATA_TYPE>> ADrivingAgent::get_bb_length_constant() const
{
    std::shared_ptr<const IValuelessConstant> bb_length_valueless_constant =
            this->get_constant_parameter(this->get_name() + ".bb_length");
    return std::static_pointer_cast<const IConstant<FP_DATA_TYPE>>(bb_length_valueless_constant);
}

std::shared_ptr<const IConstant<FP_DATA_TYPE>> ADrivingAgent::get_bb_width_constant() const
{
    std::shared_ptr<const IValuelessConstant> bb_width_valueless_constant =
            this->get_constant_parameter(this->get_name() + ".bb_width");
    return std::static_pointer_cast<const IConstant<FP_DATA_TYPE>>(bb_width_valueless_constant);
}

std::shared_ptr<const IConstant<DrivingAgentClass>> ADrivingAgent::get_road_agent_class_constant() const
{
    std::shared_ptr<const IValuelessConstant> road_agent_class_valueless_constant =
            this->get_constant_parameter(this->get_name() + ".road_agent_class");
    return std::static_pointer_cast<const IConstant<DrivingAgentClass>>(road_agent_class_valueless_constant);
}

std::shared_ptr<const IVariable<geometry::Vec>> ADrivingAgent::get_position_variable() const
{
    std::shared_ptr<const IValuelessVariable> position_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".position.base");
    return std::static_pointer_cast<const IVariable<geometry::Vec>>(position_valueless_variable);
}

std::shared_ptr<const IVariable<geometry::Vec>> ADrivingAgent::get_linear_velocity_variable() const
{
    std::shared_ptr<const IValuelessVariable> linear_velocity_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".linear_velocity.base");
    return std::static_pointer_cast<const IVariable<geometry::Vec>>(linear_velocity_valueless_variable);
}

std::shared_ptr<const IVariable<FP_DATA_TYPE>> ADrivingAgent::get_aligned_linear_velocity_variable() const
{
    std::shared_ptr<const IValuelessVariable> aligned_linear_velocity_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".aligned_linear_velocity.base");
    return std::static_pointer_cast<const IVariable<FP_DATA_TYPE>>(aligned_linear_velocity_valueless_variable);
}

std::shared_ptr<const IVariable<geometry::Vec>> ADrivingAgent::get_linear_acceleration_variable() const
{
    std::shared_ptr<const IValuelessVariable> linear_acceleration_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".linear_acceleration.base");
    return std::static_pointer_cast<const IVariable<geometry::Vec>>(linear_acceleration_valueless_variable);
}

std::shared_ptr<const IVariable<FP_DATA_TYPE>> ADrivingAgent::get_aligned_linear_acceleration_variable() const
{
    std::shared_ptr<const IValuelessVariable> aligned_linear_acceleration_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".aligned_linear_acceleration.indirect_actuation");
    return std::static_pointer_cast<const IVariable<FP_DATA_TYPE>>(aligned_linear_acceleration_valueless_variable);
}

std::shared_ptr<const IVariable<FP_DATA_TYPE>> ADrivingAgent::get_rotation_variable() const
{
    std::shared_ptr<const IValuelessVariable> rotation_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".rotation.base");
    return std::static_pointer_cast<const IVariable<FP_DATA_TYPE>>(rotation_valueless_variable);
}

std::shared_ptr<const IVariable<FP_DATA_TYPE>> ADrivingAgent::get_steer_variable() const
{
    std::shared_ptr<const IValuelessVariable> steer_valueless_variable =
            this->get_variable_parameter(this->get_name() + ".steer.indirect_actuation");
    return std::static_pointer_cast<const IVariable<FP_DATA_TYPE>>(steer_valueless_variable);
}

}
}
}
