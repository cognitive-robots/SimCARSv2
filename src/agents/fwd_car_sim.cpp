
#include <ori/simcars/agents/fwd_car_sim.hpp>

#include <ori/simcars/agents/rect_rigid_body_sim.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

FWDCarSim::FWDCarSim(FWDCar *fwd_car, temporal::Time start_time) :
    RectRigidBody(*fwd_car),
    RectRigidBodySim(fwd_car, start_time),
    FWDCar(*fwd_car),

    sim_motor_torque(fwd_car->get_motor_torque_variable(), &(this->motor_torque_buff), start_time),
    sim_steer(fwd_car->get_steer_variable(), &(this->steer_buff), start_time)
{
    dir.set_parent(&sim_rot);

    lon_lin_acc.set_endogenous_parent(&sim_lin_acc);
    lon_lin_vel.set_endogenous_parent(&sim_lin_vel);

    lat_lin_vel.set_endogenous_parent(&sim_lin_vel);

    front_wheel_lon_force_mag.set_other_parent(&sim_motor_torque);
    front_wheel_ang_lat_lin_vel.set_endogenous_parent(&sim_ang_vel);
    front_wheel_slip_ang.set_other_parent(&sim_steer);

    rear_wheel_ang_lat_lin_vel.set_endogenous_parent(&sim_ang_vel);

    steer_mat.set_parent(&sim_steer);

    rot_mat.set_parent(&sim_rot);
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* FWDCarSim::get_motor_torque_variable()
{
    return &sim_motor_torque;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* FWDCarSim::get_steer_variable()
{
    return &sim_steer;
}

}
}
}
