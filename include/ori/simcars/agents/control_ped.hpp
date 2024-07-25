#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/endogenous_variable_interface.hpp>
#include <ori/simcars/causal/variable_types/exogenous/id_socket.hpp>
#include <ori/simcars/causal/variable_types/exogenous/vector_socket.hpp>
#include <ori/simcars/agents/declarations.hpp>
#include <ori/simcars/agents/plan_ped.hpp>
#include <ori/simcars/agents/causal/variable_types/exogenous/ped_action_socket.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/id_goal_val_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/id_goal_time_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_action_node_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_action_buffer.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class ControlPed
{
protected:
    virtual void init_links() = 0;

    Ped *ped;

    causal::PedActionSocketVariable action;
    causal::PedActionBufferVariable action_buff;

    causal::PedActionNodePartVariable node_goal;
    causal::IdGoalValPartVariable node_val_goal;
    causal::IdGoalTimePartVariable node_time_goal;

    simcars::causal::VectorSocketVariable ped_force;

public:
    ControlPed();

    Ped* get_ped();

    void set_ped(Ped *ped);

    friend void PlanPed::set_control_ped(ControlPed *control_ped);
};

}
}
}
