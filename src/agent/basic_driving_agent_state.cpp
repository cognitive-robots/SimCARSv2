
#include <ori/simcars/agent/basic_constant.hpp>
#include <ori/simcars/agent/basic_driving_agent_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

BasicDrivingAgentState::BasicDrivingAgentState(const std::string& driving_agent_name) : driving_agent_name(driving_agent_name) {}

BasicDrivingAgentState::BasicDrivingAgentState(std::shared_ptr<const IDrivingAgentState> driving_agent_state) :
    driving_agent_name(driving_agent_state->get_driving_agent_name())
{
    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> parameter_values =
            driving_agent_state->get_parameter_values();
    for (size_t i = 0; i < parameter_values->count(); ++i)
    {
        parameter_dict.update((*parameter_values)[i]->get_full_name(), (*parameter_values)[i]);
    }
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> BasicDrivingAgentState::get_parameter_values() const
{
    return std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>>(
            new structures::stl::STLStackArray<std::shared_ptr<const IValuelessConstant>>(parameter_dict.get_values()));
}

std::shared_ptr<const IValuelessConstant> BasicDrivingAgentState::get_parameter_value(const std::string& parameter_name) const
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

void BasicDrivingAgentState::set_id_constant(std::shared_ptr<const IConstant<uint32_t>> id_constant)
{
    parameter_dict.update(id_constant->get_full_name(), id_constant);
}

void BasicDrivingAgentState::set_ego_constant(std::shared_ptr<const IConstant<bool>> ego_constant)
{
    parameter_dict.update(ego_constant->get_full_name(), ego_constant);
}

void BasicDrivingAgentState::set_bb_length_constant(std::shared_ptr<const IConstant<FP_DATA_TYPE>> bb_length_constant)
{
    parameter_dict.update(bb_length_constant->get_full_name(), bb_length_constant);
}

void BasicDrivingAgentState::set_bb_width_constant(std::shared_ptr<const IConstant<FP_DATA_TYPE>> bb_width_constant)
{
    parameter_dict.update(bb_width_constant->get_full_name(), bb_width_constant);
}

void BasicDrivingAgentState::set_driving_agent_class_constant(std::shared_ptr<const IConstant<DrivingAgentClass>> driving_agent_class_constant)
{
    parameter_dict.update(driving_agent_class_constant->get_full_name(), driving_agent_class_constant);
}

void BasicDrivingAgentState::set_position_variable(std::shared_ptr<const IConstant<geometry::Vec>> position_variable)
{
    parameter_dict.update(position_variable->get_full_name(), position_variable);
}

void BasicDrivingAgentState::set_linear_velocity_variable(std::shared_ptr<const IConstant<geometry::Vec>> linear_velocity_variable)
{
    parameter_dict.update(linear_velocity_variable->get_full_name(), linear_velocity_variable);
}

void BasicDrivingAgentState::set_aligned_linear_velocity_variable(std::shared_ptr<const IConstant<FP_DATA_TYPE>> aligned_linear_velocity_variable)
{
    parameter_dict.update(aligned_linear_velocity_variable->get_full_name(), aligned_linear_velocity_variable);
}

void BasicDrivingAgentState::set_linear_acceleration_variable(std::shared_ptr<const IConstant<geometry::Vec>> linear_acceleration_variable)
{
    parameter_dict.update(linear_acceleration_variable->get_full_name(), linear_acceleration_variable);
}

void BasicDrivingAgentState::set_aligned_linear_acceleration_variable(std::shared_ptr<const IConstant<FP_DATA_TYPE>> aligned_linear_acceleration_variable)
{
    parameter_dict.update(aligned_linear_acceleration_variable->get_full_name(), aligned_linear_acceleration_variable);
}

void BasicDrivingAgentState::set_external_linear_acceleration_variable(std::shared_ptr<const IConstant<geometry::Vec> > external_linear_acceleration_variable)
{
    parameter_dict.update(external_linear_acceleration_variable->get_full_name(), external_linear_acceleration_variable);
}

void BasicDrivingAgentState::set_rotation_variable(std::shared_ptr<const IConstant<FP_DATA_TYPE>> rotation_variable)
{
    parameter_dict.update(rotation_variable->get_full_name(), rotation_variable);
}

void BasicDrivingAgentState::set_steer_variable(std::shared_ptr<const IConstant<FP_DATA_TYPE>> steer_variable)
{
    parameter_dict.update(steer_variable->get_full_name(), steer_variable);
}

void BasicDrivingAgentState::set_angular_velocity_variable(std::shared_ptr<const IConstant<FP_DATA_TYPE>> angular_velocity_variable)
{
    parameter_dict.update(angular_velocity_variable->get_full_name(), angular_velocity_variable);
}

}
}
}
