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
    GoalDrivingAgentState(const std::string& driving_agent_name);
    GoalDrivingAgentState(std::shared_ptr<const IDrivingAgentState> driving_agent_state);

    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_aligned_linear_velocity_goal_value_variable() const;
    virtual std::shared_ptr<const IConstant<temporal::Duration>> get_aligned_linear_velocity_goal_duration_variable() const;

    virtual void set_aligned_linear_velocity_goal_value_variable(
            std::shared_ptr<const IConstant<FP_DATA_TYPE>> aligned_linear_velocity_goal_value_variable);
    virtual void set_aligned_linear_velocity_goal_duration_variable(
            std::shared_ptr<const IConstant<temporal::Duration>> aligned_linear_velocity_goal_duration_variable);
};

}
}
}
