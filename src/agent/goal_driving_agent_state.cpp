
#include <ori/simcars/agent/goal_driving_agent_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

GoalDrivingAgentState::GoalDrivingAgentState(const std::string& driving_agent_name) :
    BasicDrivingAgentState(driving_agent_name) {}

GoalDrivingAgentState::GoalDrivingAgentState(std::shared_ptr<const IDrivingAgentState> driving_agent_state) :
    BasicDrivingAgentState(driving_agent_state) {}

std::shared_ptr<const IConstant<FP_DATA_TYPE>> GoalDrivingAgentState::get_aligned_linear_velocity_goal_value_variable() const
{
    std::shared_ptr<const IValuelessConstant> aligned_linear_velocity_goal_value_valueless_variable =
            this->get_parameter_value(this->get_driving_agent_name() + ".aligned_linear_velocity.goal_value");
    return std::dynamic_pointer_cast<const IConstant<FP_DATA_TYPE>>(aligned_linear_velocity_goal_value_valueless_variable);
}

std::shared_ptr<const IConstant<temporal::Duration>> GoalDrivingAgentState::get_aligned_linear_velocity_goal_duration_variable() const
{
    std::shared_ptr<const IValuelessConstant> aligned_linear_velocity_goal_duration_valueless_variable =
            this->get_parameter_value(this->get_driving_agent_name() + ".aligned_linear_velocity.goal_duration");
    return std::dynamic_pointer_cast<const IConstant<temporal::Duration>>(
                aligned_linear_velocity_goal_duration_valueless_variable);
}

void GoalDrivingAgentState::set_aligned_linear_velocity_goal_value_variable(
        std::shared_ptr<const IConstant<FP_DATA_TYPE>> aligned_linear_velocity_goal_value_variable)
{
    parameter_dict.update(aligned_linear_velocity_goal_value_variable->get_full_name(),
                          aligned_linear_velocity_goal_value_variable);
}

void GoalDrivingAgentState::set_aligned_linear_velocity_goal_duration_variable(
        std::shared_ptr<const IConstant<temporal::Duration>> aligned_linear_velocity_goal_duration_variable)
{
    parameter_dict.update(aligned_linear_velocity_goal_duration_variable->get_full_name(),
                          aligned_linear_velocity_goal_duration_variable);
}

}
}
}
