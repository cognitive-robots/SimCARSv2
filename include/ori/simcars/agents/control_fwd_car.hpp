#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/endogenous_variable_interface.hpp>
#include <ori/simcars/causal/variable_types/exogenous/id_socket.hpp>
#include <ori/simcars/causal/variable_types/exogenous/scalar_socket.hpp>
#include <ori/simcars/causal/variable_types/exogenous/time_socket.hpp>
#include <ori/simcars/agents/declarations.hpp>
#include <ori/simcars/agents/plan_fwd_car.hpp>
#include <ori/simcars/agents/causal/variable_types/exogenous/fwd_car_action_socket.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/scalar_goal_val_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/scalar_goal_time_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/id_goal_val_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/id_goal_time_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_speed_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_lane_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_buffer.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class ControlFWDCar
{
protected:
    virtual void init_links() = 0;

    FWDCar *fwd_car;

    causal::FWDCarActionSocketVariable action;
    causal::FWDCarActionBufferVariable action_buff;

    causal::FWDCarActionSpeedPartVariable speed_goal;
    causal::ScalarGoalValPartVariable speed_val_goal;
    causal::ScalarGoalTimePartVariable speed_time_goal;

    causal::FWDCarActionLanePartVariable lane_goal;
    causal::IdGoalValPartVariable lane_val_goal;
    causal::IdGoalTimePartVariable lane_time_goal;


    simcars::causal::ScalarSocketVariable motor_torque;
    simcars::causal::ScalarSocketVariable steer;

public:
    ControlFWDCar();

    FWDCar* get_fwd_car();

    void set_fwd_car(FWDCar *fwd_car);

    friend void PlanFWDCar::set_control_fwd_car(ControlFWDCar *control_fwd_car);
};

}
}
}
