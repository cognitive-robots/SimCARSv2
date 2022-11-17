
#include <ori/simcars/agent/goal_driving_agent_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

GoalDrivingAgentState::GoalDrivingAgentState(std::string const &driving_agent_name) :
    BasicDrivingAgentState(driving_agent_name) {}

GoalDrivingAgentState::GoalDrivingAgentState(IDrivingAgentState const *driving_agent_state) :
    BasicDrivingAgentState(driving_agent_state) {}

IConstant<FP_DATA_TYPE> const* GoalDrivingAgentState::get_aligned_linear_velocity_goal_value_variable() const
{
    IValuelessConstant const *aligned_linear_velocity_goal_value_valueless_variable =
            this->get_parameter_value(this->get_driving_agent_name() + ".aligned_linear_velocity.goal_value");
    return dynamic_cast<IConstant<FP_DATA_TYPE> const*>(aligned_linear_velocity_goal_value_valueless_variable);
}

IConstant<temporal::Duration> const* GoalDrivingAgentState::get_aligned_linear_velocity_goal_duration_variable() const
{
    IValuelessConstant const *aligned_linear_velocity_goal_duration_valueless_variable =
            this->get_parameter_value(this->get_driving_agent_name() + ".aligned_linear_velocity.goal_duration");
    return dynamic_cast<IConstant<temporal::Duration> const*>(
                aligned_linear_velocity_goal_duration_valueless_variable);
}

void GoalDrivingAgentState::set_aligned_linear_velocity_goal_value_variable(
        IConstant<FP_DATA_TYPE> const *aligned_linear_velocity_goal_value_variable)
{
    parameter_dict.update(aligned_linear_velocity_goal_value_variable->get_full_name(),
                          aligned_linear_velocity_goal_value_variable);
}

void GoalDrivingAgentState::set_aligned_linear_velocity_goal_duration_variable(
        IConstant<temporal::Duration> const *aligned_linear_velocity_goal_duration_variable)
{
    parameter_dict.update(aligned_linear_velocity_goal_duration_variable->get_full_name(),
                          aligned_linear_velocity_goal_duration_variable);
}

}
}
}
