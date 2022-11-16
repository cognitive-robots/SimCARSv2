
#include <ori/simcars/agent/basic_constant.hpp>
#include <ori/simcars/agent/basic_driving_agent_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

BasicDrivingAgentState::BasicDrivingAgentState(const std::string& driving_agent_name) : driving_agent_name(driving_agent_name) {}

BasicDrivingAgentState::BasicDrivingAgentState(IDrivingAgentState const *driving_agent_state) :
    driving_agent_name(driving_agent_state->get_driving_agent_name())
{
    structures::IArray<IValuelessConstant const*> *parameter_values =
            driving_agent_state->get_parameter_values();
    for (size_t i = 0; i < parameter_values->count(); ++i)
    {
        parameter_dict.update((*parameter_values)[i]->get_full_name(), (*parameter_values)[i]);
    }
}

structures::IArray<IValuelessConstant const*>* BasicDrivingAgentState::get_parameter_values() const
{
    return new structures::stl::STLStackArray<IValuelessConstant const*>(parameter_dict.get_values());
}

IValuelessConstant const* BasicDrivingAgentState::get_parameter_value(const std::string& parameter_name) const
{
    return parameter_dict[parameter_name];
}

std::string BasicDrivingAgentState::get_driving_agent_name() const
{
    return driving_agent_name;
}

void BasicDrivingAgentState::set_driving_agent_name(const std::string& driving_agent_name)
{
    this->driving_agent_name = driving_agent_name;
}

void BasicDrivingAgentState::set_id_constant(IConstant<uint32_t> const *id_constant)
{
    parameter_dict.update(this->get_driving_agent_name() + ".id", id_constant);
}

void BasicDrivingAgentState::set_ego_constant(IConstant<bool> const *ego_constant)
{
    parameter_dict.update(this->get_driving_agent_name() + ".ego", ego_constant);
}

void BasicDrivingAgentState::set_bb_length_constant(IConstant<FP_DATA_TYPE> const *bb_length_constant)
{
    parameter_dict.update(this->get_driving_agent_name() + ".bb_length", bb_length_constant);
}

void BasicDrivingAgentState::set_bb_width_constant(IConstant<FP_DATA_TYPE> const *bb_width_constant)
{
    parameter_dict.update(this->get_driving_agent_name() + ".bb_width", bb_width_constant);
}

void BasicDrivingAgentState::set_driving_agent_class_constant(IConstant<DrivingAgentClass> const *driving_agent_class_constant)
{
    parameter_dict.update(this->get_driving_agent_name() + ".driving_agent_class", driving_agent_class_constant);
}

void BasicDrivingAgentState::set_position_variable(IConstant<geometry::Vec> const *position_variable)
{
    parameter_dict.update(this->get_driving_agent_name() + ".position.base", position_variable);
}

void BasicDrivingAgentState::set_linear_velocity_variable(IConstant<geometry::Vec> const *linear_velocity_variable)
{
    parameter_dict.update(this->get_driving_agent_name() + ".linear_velocity.base", linear_velocity_variable);
}

void BasicDrivingAgentState::set_aligned_linear_velocity_variable(IConstant<FP_DATA_TYPE> const *aligned_linear_velocity_variable)
{
    parameter_dict.update(this->get_driving_agent_name() + ".aligned_linear_velocity.base", aligned_linear_velocity_variable);
}

void BasicDrivingAgentState::set_linear_acceleration_variable(IConstant<geometry::Vec> const *linear_acceleration_variable)
{
    parameter_dict.update(this->get_driving_agent_name() + ".linear_acceleration.base", linear_acceleration_variable);
}

void BasicDrivingAgentState::set_aligned_linear_acceleration_variable(IConstant<FP_DATA_TYPE> const *aligned_linear_acceleration_variable)
{
    parameter_dict.update(this->get_driving_agent_name() + ".aligned_linear_acceleration.indirect_actuation", aligned_linear_acceleration_variable);
}

void BasicDrivingAgentState::set_external_linear_acceleration_variable(IConstant<geometry::Vec> const *external_linear_acceleration_variable)
{
    parameter_dict.update(this->get_driving_agent_name() + ".linear_acceleration.external", external_linear_acceleration_variable);
}

void BasicDrivingAgentState::set_rotation_variable(IConstant<FP_DATA_TYPE> const *rotation_variable)
{
    parameter_dict.update(this->get_driving_agent_name() + ".rotation.base", rotation_variable);
}

void BasicDrivingAgentState::set_steer_variable(IConstant<FP_DATA_TYPE> const *steer_variable)
{
    parameter_dict.update(this->get_driving_agent_name() + ".steer.indirect_actuation", steer_variable);
}

void BasicDrivingAgentState::set_angular_velocity_variable(IConstant<FP_DATA_TYPE> const *angular_velocity_variable)
{
    parameter_dict.update(this->get_driving_agent_name() + ".angular_velocity.base", angular_velocity_variable);
}

}
}
}
