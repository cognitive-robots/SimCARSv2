
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/basic_event.hpp>
#include <ori/simcars/agent/view_driving_agent_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

ViewDrivingAgentState::ViewDrivingAgentState(IDrivingAgent *agent, temporal::Time time)
    : agent(agent), time(time)
{
}

std::string ViewDrivingAgentState::get_name() const
{
    return agent->get_name();
}

temporal::Time ViewDrivingAgentState::get_time() const
{
    return time;
}

// Is not the best approach to check if state is populated, relies upon fact
// that simulator updates position variable last
bool ViewDrivingAgentState::is_populated() const
{
    return agent->get_position_variable()->has_event(time);
}

// Inefficient, use other access functions if you can
structures::IArray<IValuelessConstant const*>* ViewDrivingAgentState::get_parameter_values() const
{
    structures::IStackArray<IValuelessConstant const*> *parameter_values =
            new structures::stl::STLStackArray<IValuelessConstant const*>;

    size_t i;

    structures::IArray<IValuelessConstant const*> *constants =
            agent->get_constant_parameters();
    for (i = 0; i < constants->count(); ++i)
    {
        parameter_values->push_back((*constants)[i]);
    }

    structures::IArray<IValuelessVariable const*> *variables =
            agent->get_variable_parameters();
    for (i = 0; i < variables->count(); ++i)
    {
        parameter_values->push_back((*variables)[i]->get_valueless_event(time));
    }

    return parameter_values;
}

// Inefficient, use other access functions if you can
IValuelessConstant const* ViewDrivingAgentState::get_parameter_value(std::string const &parameter_name) const
{
    IValuelessConstant const *parameter_value;
    parameter_value = agent->get_constant_parameter(parameter_name);
    if (parameter_value != nullptr)
    {
        return parameter_value;
    }
    else
    {
        IValuelessVariable const *valueless_variable =
                agent->get_variable_parameter(parameter_name);
        if (valueless_variable != nullptr)
        {
            return valueless_variable->get_valueless_event(time);
        }
        else
        {
            return nullptr;
        }
    }
}

IConstant<uint32_t> const* ViewDrivingAgentState::get_id_constant() const
{
    return agent->get_id_constant();
}

IConstant<bool> const* ViewDrivingAgentState::get_ego_constant() const
{
    return agent->get_ego_constant();
}

IConstant<FP_DATA_TYPE> const* ViewDrivingAgentState::get_bb_length_constant() const
{
    return agent->get_bb_length_constant();
}

IConstant<FP_DATA_TYPE> const* ViewDrivingAgentState::get_bb_width_constant() const
{
    return agent->get_bb_width_constant();
}

IConstant<DrivingAgentClass> const* ViewDrivingAgentState::get_driving_agent_class_constant() const
{
    return agent->get_driving_agent_class_constant();
}

IConstant<geometry::Vec> const* ViewDrivingAgentState::get_position_variable() const
{
    return agent->get_position_variable()->get_event(time);
}

IConstant<geometry::Vec> const* ViewDrivingAgentState::get_linear_velocity_variable() const
{
    return agent->get_linear_velocity_variable()->get_event(time);
}

IConstant<FP_DATA_TYPE> const* ViewDrivingAgentState::get_aligned_linear_velocity_variable() const
{
    return agent->get_aligned_linear_velocity_variable()->get_event(time);
}

IConstant<geometry::Vec> const* ViewDrivingAgentState::get_linear_acceleration_variable() const
{
    return agent->get_linear_acceleration_variable()->get_event(time);
}

IConstant<FP_DATA_TYPE> const* ViewDrivingAgentState::get_aligned_linear_acceleration_variable() const
{
    return agent->get_aligned_linear_acceleration_variable()->get_event(time);
}

IConstant<geometry::Vec> const* ViewDrivingAgentState::get_external_linear_acceleration_variable() const
{
    return agent->get_external_linear_acceleration_variable()->get_event(time);
}

IConstant<FP_DATA_TYPE> const* ViewDrivingAgentState::get_rotation_variable() const
{
    return agent->get_rotation_variable()->get_event(time);
}

IConstant<FP_DATA_TYPE> const* ViewDrivingAgentState::get_steer_variable() const
{
    return agent->get_steer_variable()->get_event(time);
}

IConstant<FP_DATA_TYPE> const* ViewDrivingAgentState::get_angular_velocity_variable() const
{
    return agent->get_angular_velocity_variable()->get_event(time);
}

IDrivingAgent const* ViewDrivingAgentState::get_agent() const
{
    return agent;
}

// Inefficient, use other access functions if you can
structures::IArray<IValuelessConstant*>* ViewDrivingAgentState::get_mutable_parameter_values()
{
    structures::IStackArray<IValuelessConstant*> *parameter_values =
            new structures::stl::STLStackArray<IValuelessConstant*>;

    size_t i;

    structures::IArray<IValuelessConstant*> *constants =
            agent->get_mutable_constant_parameters();
    for (i = 0; i < constants->count(); ++i)
    {
        parameter_values->push_back((*constants)[i]);
    }

    structures::IArray<IValuelessVariable*> *variables =
            agent->get_mutable_variable_parameters();
    for (i = 0; i < variables->count(); ++i)
    {
        parameter_values->push_back((*variables)[i]->get_mutable_valueless_event(time));
    }

    return parameter_values;
}

// Inefficient, use other access functions if you can
IValuelessConstant* ViewDrivingAgentState::get_mutable_parameter_value(std::string const &parameter_name)
{
    IValuelessConstant *parameter_value;
    parameter_value = agent->get_mutable_constant_parameter(parameter_name);
    if (parameter_value != nullptr)
    {
        return parameter_value;
    }
    else
    {
        IValuelessVariable *valueless_variable =
                agent->get_mutable_variable_parameter(parameter_name);
        if (valueless_variable != nullptr)
        {
            return valueless_variable->get_mutable_valueless_event(time);
        }
        else
        {
            return nullptr;
        }
    }
}

void ViewDrivingAgentState::set_id_constant(IConstant<uint32_t> *id_constant)
{
    agent->get_mutable_id_constant()->set_value(id_constant->get_value());
    delete id_constant;
}

void ViewDrivingAgentState::set_ego_constant(IConstant<bool> *ego_constant)
{
    agent->get_mutable_ego_constant()->set_value(ego_constant->get_value());
    delete ego_constant;
}

void ViewDrivingAgentState::set_bb_length_constant(IConstant<FP_DATA_TYPE> *bb_length_constant)
{
    agent->get_mutable_bb_length_constant()->set_value(bb_length_constant->get_value());
    delete bb_length_constant;
}

void ViewDrivingAgentState::set_bb_width_constant(IConstant<FP_DATA_TYPE> *bb_width_constant)
{
    agent->get_mutable_bb_width_constant()->set_value(bb_width_constant->get_value());
    delete bb_width_constant;
}

void ViewDrivingAgentState::set_driving_agent_class_constant(IConstant<DrivingAgentClass> *driving_agent_class_constant)
{
    agent->get_mutable_driving_agent_class_constant()->set_value(driving_agent_class_constant->get_value());
    delete driving_agent_class_constant;
}

void ViewDrivingAgentState::set_position_variable(IConstant<geometry::Vec> *position_variable)
{
    agent->get_mutable_position_variable()->set_value(time, position_variable->get_value());
    delete position_variable;
}

void ViewDrivingAgentState::set_linear_velocity_variable(IConstant<geometry::Vec> *linear_velocity_variable)
{
    agent->get_mutable_linear_velocity_variable()->set_value(time, linear_velocity_variable->get_value());
    delete linear_velocity_variable;
}

void ViewDrivingAgentState::set_aligned_linear_velocity_variable(IConstant<FP_DATA_TYPE> *aligned_linear_velocity_variable)
{
    agent->get_mutable_aligned_linear_velocity_variable()->set_value(time, aligned_linear_velocity_variable->get_value());
    delete aligned_linear_velocity_variable;
}

void ViewDrivingAgentState::set_linear_acceleration_variable(IConstant<geometry::Vec> *linear_acceleration_variable)
{
    agent->get_mutable_linear_acceleration_variable()->set_value(time, linear_acceleration_variable->get_value());
    delete linear_acceleration_variable;
}

void ViewDrivingAgentState::set_aligned_linear_acceleration_variable(IConstant<FP_DATA_TYPE> *aligned_linear_acceleration_variable)
{
    agent->get_mutable_aligned_linear_acceleration_variable()->set_value(time, aligned_linear_acceleration_variable->get_value());
    delete aligned_linear_acceleration_variable;
}

void ViewDrivingAgentState::set_external_linear_acceleration_variable(IConstant<geometry::Vec> *external_linear_acceleration_variable)
{
    agent->get_mutable_external_linear_acceleration_variable()->set_value(time, external_linear_acceleration_variable->get_value());
    delete external_linear_acceleration_variable;
}

void ViewDrivingAgentState::set_rotation_variable(IConstant<FP_DATA_TYPE> *rotation_variable)
{
    agent->get_mutable_rotation_variable()->set_value(time, rotation_variable->get_value());
    delete rotation_variable;
}

void ViewDrivingAgentState::set_steer_variable(IConstant<FP_DATA_TYPE> *steer_variable)
{
    agent->get_mutable_steer_variable()->set_value(time, steer_variable->get_value());
    delete steer_variable;
}

void ViewDrivingAgentState::set_angular_velocity_variable(IConstant<FP_DATA_TYPE> *angular_velocity_variable)
{
    agent->get_mutable_angular_velocity_variable()->set_value(time, angular_velocity_variable->get_value());
    delete angular_velocity_variable;
}

void ViewDrivingAgentState::set_agent(IDrivingAgent *agent)
{
    this->agent = agent;
}

void ViewDrivingAgentState::set_time(temporal::Time time)
{
    this->time = time;
}

}
}
}
