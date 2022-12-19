
#include <ori/simcars/agent/basic_goal_driving_agent_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

BasicGoalDrivingAgentState::BasicGoalDrivingAgentState(std::string const &driving_agent_name, temporal::Time time, bool delete_dicts) :
    BasicDrivingAgentState(driving_agent_name, time, delete_dicts) {}

BasicGoalDrivingAgentState::BasicGoalDrivingAgentState(IDrivingAgentState *driving_agent_state, bool copy_parameters) :
    BasicDrivingAgentState(driving_agent_state, copy_parameters) {}

IConstant<Goal<FP_DATA_TYPE>> const* BasicGoalDrivingAgentState::get_aligned_linear_velocity_goal_variable() const
{
    IValuelessConstant const *aligned_linear_velocity_goal_valueless_variable =
            this->get_parameter_value(this->get_name() + ".aligned_linear_velocity.goal");
    return dynamic_cast<IConstant<Goal<FP_DATA_TYPE>> const*>(aligned_linear_velocity_goal_valueless_variable);
}

void BasicGoalDrivingAgentState::set_aligned_linear_velocity_goal_variable(
        IConstant<Goal<FP_DATA_TYPE>> *aligned_linear_velocity_goal_variable)
{
    parameter_dict.update(aligned_linear_velocity_goal_variable->get_full_name(),
                          aligned_linear_velocity_goal_variable);
}

}
}
}
