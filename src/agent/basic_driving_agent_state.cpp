
#include <ori/simcars/agent/basic_constant.hpp>
#include <ori/simcars/agent/basic_driving_agent_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

BasicDrivingAgentState::BasicDrivingAgentState(std::string const &driving_agent_name) : driving_agent_name(driving_agent_name) {}

BasicDrivingAgentState::BasicDrivingAgentState(IDrivingAgentState const *driving_agent_state) :
    driving_agent_name(driving_agent_state->get_driving_agent_name())
{
    structures::IArray<IValuelessConstant const*> *parameter_values =
            driving_agent_state->get_parameter_values();

    for (size_t i = 0; i < parameter_values->count(); ++i)
    {
        parameter_dict.update((*parameter_values)[i]->get_full_name(), (*parameter_values)[i]->valueless_shallow_copy());
    }

    delete parameter_values;
}

BasicDrivingAgentState::~BasicDrivingAgentState()
{
    structures::IArray<IValuelessConstant const*> const *parameters = parameter_dict.get_values();

    for (size_t i = 0; i < parameters->count(); ++i)
    {
        delete (*parameters)[i];
    }
}

structures::IArray<IValuelessConstant const*>* BasicDrivingAgentState::get_parameter_values() const
{
    structures::IStackArray<IValuelessConstant const*> *parameters =
            new structures::stl::STLStackArray<IValuelessConstant const*>(parameter_dict.count());
    parameter_dict.get_values(parameters);
    return parameters;
}

IValuelessConstant const* BasicDrivingAgentState::get_parameter_value(std::string const &parameter_name) const
{
    return parameter_dict[parameter_name];
}

void BasicDrivingAgentState::set_parameter_values(structures::IArray<IValuelessConstant const*> const *parameter_values)
{
    for (size_t i = 0; i < parameter_values->count(); ++i)
    {
        set_parameter_value((*parameter_values)[i]);
    }
}

void BasicDrivingAgentState::set_parameter_value(IValuelessConstant const *parameter_value)
{
    std::string const &parameter_name = parameter_value->get_full_name();
    if (parameter_dict.contains(parameter_name))
    {
        delete parameter_dict[parameter_name];
    }
    parameter_dict.update(parameter_name, parameter_value);
}

std::string BasicDrivingAgentState::get_driving_agent_name() const
{
    return driving_agent_name;
}

void BasicDrivingAgentState::set_driving_agent_name(std::string const &driving_agent_name)
{
    this->driving_agent_name = driving_agent_name;
}

void BasicDrivingAgentState::set_id_constant(IConstant<uint32_t> const *id_constant)
{
    if (id_constant->get_entity_name() == this->get_driving_agent_name() &&
            id_constant->get_parameter_name() == "id")
    {
        this->set_parameter_value(id_constant);
    }
}

void BasicDrivingAgentState::set_ego_constant(IConstant<bool> const *ego_constant)
{
    if (ego_constant->get_entity_name() == this->get_driving_agent_name() &&
            ego_constant->get_parameter_name() == "ego")
    {
        this->set_parameter_value(ego_constant);
    }
}

void BasicDrivingAgentState::set_bb_length_constant(IConstant<FP_DATA_TYPE> const *bb_length_constant)
{
    if (bb_length_constant->get_entity_name() == this->get_driving_agent_name() &&
            bb_length_constant->get_parameter_name() == "bb_length")
    {
        this->set_parameter_value(bb_length_constant);
    }
}

void BasicDrivingAgentState::set_bb_width_constant(IConstant<FP_DATA_TYPE> const *bb_width_constant)
{
    if (bb_width_constant->get_entity_name() == this->get_driving_agent_name() &&
            bb_width_constant->get_parameter_name() == "bb_width")
    {
        this->set_parameter_value(bb_width_constant);
    }
}

void BasicDrivingAgentState::set_driving_agent_class_constant(IConstant<DrivingAgentClass> const *driving_agent_class_constant)
{
    if (driving_agent_class_constant->get_entity_name() == this->get_driving_agent_name() &&
            driving_agent_class_constant->get_parameter_name() == "driving_agent_class")
    {
        this->set_parameter_value(driving_agent_class_constant);
    }
}

void BasicDrivingAgentState::set_position_variable(IConstant<geometry::Vec> const *position_variable)
{
    if (position_variable->get_entity_name() == this->get_driving_agent_name() &&
            position_variable->get_parameter_name() == "position.base")
    {
        this->set_parameter_value(position_variable);
    }
}

void BasicDrivingAgentState::set_linear_velocity_variable(IConstant<geometry::Vec> const *linear_velocity_variable)
{
    if (linear_velocity_variable->get_entity_name() == this->get_driving_agent_name() &&
            linear_velocity_variable->get_parameter_name() == "linear_velocity.base")
    {
        this->set_parameter_value(linear_velocity_variable);
    }
}

void BasicDrivingAgentState::set_aligned_linear_velocity_variable(IConstant<FP_DATA_TYPE> const *aligned_linear_velocity_variable)
{
    if (aligned_linear_velocity_variable->get_entity_name() == this->get_driving_agent_name() &&
            aligned_linear_velocity_variable->get_parameter_name() == "aligned_linear_velocity.base")
    {
        this->set_parameter_value(aligned_linear_velocity_variable);
    }
}

void BasicDrivingAgentState::set_linear_acceleration_variable(IConstant<geometry::Vec> const *linear_acceleration_variable)
{
    if (linear_acceleration_variable->get_entity_name() == this->get_driving_agent_name() &&
            linear_acceleration_variable->get_parameter_name() == "linear_acceleration.base")
    {
        this->set_parameter_value(linear_acceleration_variable);
    }
}

void BasicDrivingAgentState::set_aligned_linear_acceleration_variable(IConstant<FP_DATA_TYPE> const *aligned_linear_acceleration_variable)
{
    if (aligned_linear_acceleration_variable->get_entity_name() == this->get_driving_agent_name() &&
            aligned_linear_acceleration_variable->get_parameter_name() == "aligned_linear_acceleration.indirect_actuation")
    {
        this->set_parameter_value(aligned_linear_acceleration_variable);
    }
}

void BasicDrivingAgentState::set_external_linear_acceleration_variable(IConstant<geometry::Vec> const *external_linear_acceleration_variable)
{
    if (external_linear_acceleration_variable->get_entity_name() == this->get_driving_agent_name() &&
            external_linear_acceleration_variable->get_parameter_name() == "linear_acceleration.external")
    {
        this->set_parameter_value(external_linear_acceleration_variable);
    }
}

void BasicDrivingAgentState::set_rotation_variable(IConstant<FP_DATA_TYPE> const *rotation_variable)
{
    if (rotation_variable->get_entity_name() == this->get_driving_agent_name() &&
            rotation_variable->get_parameter_name() == "rotation.base")
    {
        this->set_parameter_value(rotation_variable);
    }
}

void BasicDrivingAgentState::set_steer_variable(IConstant<FP_DATA_TYPE> const *steer_variable)
{
    if (steer_variable->get_entity_name() == this->get_driving_agent_name() &&
            steer_variable->get_parameter_name() == "steer.indirect_actuation")
    {
        this->set_parameter_value(steer_variable);
    }
}

void BasicDrivingAgentState::set_angular_velocity_variable(IConstant<FP_DATA_TYPE> const *angular_velocity_variable)
{
    if (angular_velocity_variable->get_entity_name() == this->get_driving_agent_name() &&
            angular_velocity_variable->get_parameter_name() == "angular_velocity.base")
    {
        this->set_parameter_value(angular_velocity_variable);
    }
}

}
}
}
