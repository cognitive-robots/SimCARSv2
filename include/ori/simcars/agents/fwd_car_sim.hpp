#pragma once

#include <ori/simcars/agents/rect_rigid_body_sim.hpp>
#include <ori/simcars/agents/fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class FWDCarSim : public FWDCar, public RectRigidBodySim
{
protected:
    simcars::causal::ScalarTimeConditionalVariable sim_motor_torque;
    simcars::causal::ScalarTimeConditionalVariable sim_steer;

public:
    FWDCarSim(FWDCar *fwd_car, temporal::Time start_time);

    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_motor_torque_variable() override;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_steer_variable() override;
};

}
}
}
