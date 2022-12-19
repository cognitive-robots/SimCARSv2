#pragma once

#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agent/goal.hpp>
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

    virtual IConstant<Goal<FP_DATA_TYPE>> const* get_aligned_linear_velocity_goal_variable() const;

    virtual void set_aligned_linear_velocity_goal_variable(
            IConstant<Goal<FP_DATA_TYPE>>* aligned_linear_velocity_goal_variable);
};

}
}
}
