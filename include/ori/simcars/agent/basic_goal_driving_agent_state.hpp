#pragma once

#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agent/basic_driving_agent_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class BasicGoalDrivingAgentState : public virtual BasicDrivingAgentState
{
public:
    BasicGoalDrivingAgentState(std::string const &name, temporal::Time time, bool delete_dicts = true);
    BasicGoalDrivingAgentState(IDrivingAgentState *driving_agent_state, bool copy_parameters = true);

    virtual IConstant<FP_DATA_TYPE> const* get_aligned_linear_velocity_goal_value_variable() const;
    virtual IConstant<temporal::Duration> const* get_aligned_linear_velocity_goal_duration_variable() const;

    virtual void set_aligned_linear_velocity_goal_value_variable(
            IConstant<FP_DATA_TYPE>* aligned_linear_velocity_goal_value_variable);
    virtual void set_aligned_linear_velocity_goal_duration_variable(
            IConstant<temporal::Duration>* aligned_linear_velocity_goal_duration_variable);
};

}
}
}
