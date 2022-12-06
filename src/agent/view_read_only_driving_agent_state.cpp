
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/basic_event.hpp>
#include <ori/simcars/agent/view_read_only_driving_agent_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

ViewReadOnlyDrivingAgentState::ViewReadOnlyDrivingAgentState(IDrivingAgent const *agent, temporal::Time time)
    : agent(agent), time(time)
{
}

std::string ViewReadOnlyDrivingAgentState::get_name() const
{
    return agent->get_name();
}

// Is not the best approach to check if state is populated, relies upon fact
// that simulator updates position variable last
bool ViewReadOnlyDrivingAgentState::is_populated() const
{
    return agent->get_position_variable()->has_event(time);
}

// Inefficient, use other access functions if you can
structures::IArray<IValuelessConstant const*>* ViewReadOnlyDrivingAgentState::get_parameter_values() const
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
IValuelessConstant const* ViewReadOnlyDrivingAgentState::get_parameter_value(std::string const &parameter_name) const
{
    IValuelessConstant const *parameter_value;
    parameter_value = agent->get_constant_parameter(parameter_name);
    if (parameter_value != nullptr)
    {
        return parameter_value;
    }
    else
    {
        return agent->get_variable_parameter(parameter_name)->get_valueless_event(time);
    }
}

IConstant<uint32_t> const* ViewReadOnlyDrivingAgentState::get_id_constant() const
{
    return agent->get_id_constant();
}

IConstant<bool> const* ViewReadOnlyDrivingAgentState::get_ego_constant() const
{
    return agent->get_ego_constant();
}

IConstant<FP_DATA_TYPE> const* ViewReadOnlyDrivingAgentState::get_bb_length_constant() const
{
    return agent->get_bb_length_constant();
}

IConstant<FP_DATA_TYPE> const* ViewReadOnlyDrivingAgentState::get_bb_width_constant() const
{
    return agent->get_bb_width_constant();
}

IConstant<DrivingAgentClass> const* ViewReadOnlyDrivingAgentState::get_driving_agent_class_constant() const
{
    return agent->get_driving_agent_class_constant();
}

IConstant<geometry::Vec> const* ViewReadOnlyDrivingAgentState::get_position_variable() const
{
    return agent->get_position_variable()->get_event(time);
}

IConstant<geometry::Vec> const* ViewReadOnlyDrivingAgentState::get_linear_velocity_variable() const
{
    return agent->get_linear_velocity_variable()->get_event(time);
}

IConstant<FP_DATA_TYPE> const* ViewReadOnlyDrivingAgentState::get_aligned_linear_velocity_variable() const
{
    return agent->get_aligned_linear_velocity_variable()->get_event(time);
}

IConstant<geometry::Vec> const* ViewReadOnlyDrivingAgentState::get_linear_acceleration_variable() const
{
    return agent->get_linear_acceleration_variable()->get_event(time);
}

IConstant<FP_DATA_TYPE> const* ViewReadOnlyDrivingAgentState::get_aligned_linear_acceleration_variable() const
{
    return agent->get_aligned_linear_acceleration_variable()->get_event(time);
}

IConstant<geometry::Vec> const* ViewReadOnlyDrivingAgentState::get_external_linear_acceleration_variable() const
{
    return agent->get_external_linear_acceleration_variable()->get_event(time);
}

IConstant<FP_DATA_TYPE> const* ViewReadOnlyDrivingAgentState::get_rotation_variable() const
{
    return agent->get_rotation_variable()->get_event(time);
}

IConstant<FP_DATA_TYPE> const* ViewReadOnlyDrivingAgentState::get_steer_variable() const
{
    return agent->get_steer_variable()->get_event(time);
}

IConstant<FP_DATA_TYPE> const* ViewReadOnlyDrivingAgentState::get_angular_velocity_variable() const
{
    return agent->get_angular_velocity_variable()->get_event(time);
}

IDrivingAgent const* ViewReadOnlyDrivingAgentState::get_agent() const
{
    return agent;
}

temporal::Time ViewReadOnlyDrivingAgentState::get_time() const
{
    return time;
}

}
}
}
