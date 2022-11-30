#pragma once

#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agent/basic_driving_agent_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class GoalDrivingAgentState : public virtual BasicDrivingAgentState
{
public:
    GoalDrivingAgentState(std::string const &driving_agent_name, bool delete_dicts = true);
    GoalDrivingAgentState(IDrivingAgentState const *driving_agent_state, bool copy_parameters = true);

    virtual IConstant<FP_DATA_TYPE> const* get_aligned_linear_velocity_goal_value_variable() const;
    virtual IConstant<temporal::Duration> const* get_aligned_linear_velocity_goal_duration_variable() const;

    virtual void set_aligned_linear_velocity_goal_value_variable(
            IConstant<FP_DATA_TYPE> const* aligned_linear_velocity_goal_value_variable);
    virtual void set_aligned_linear_velocity_goal_duration_variable(
            IConstant<temporal::Duration> const* aligned_linear_velocity_goal_duration_variable);
};

}
}
}
