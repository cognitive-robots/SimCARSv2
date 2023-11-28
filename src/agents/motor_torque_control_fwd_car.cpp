
#include <ori/simcars/agents/motor_torque_control_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

MotorTorqueControlFWDCar::MotorTorqueControlFWDCar(FP_DATA_TYPE max_motor_torque_value,
                                                   FP_DATA_TYPE min_motor_torque_value) :
    max_motor_torque(max_motor_torque_value),
    min_motor_torque(min_motor_torque_value),

    lon_lin_vel_time_goal(),
    lon_lin_vel_val_goal(),

    time_error(&lon_lin_vel_time_goal),
    time_error_secs(&time_error),
    min_act_horizon_secs(1.0),
    actual_act_horizon_secs(&time_error_secs, &min_act_horizon_secs),
    actual_act_horizon_secs_recip(&actual_act_horizon_secs),

    lon_lin_vel(),
    neg_lon_lin_vel(&lon_lin_vel),
    lon_lin_vel_error(&neg_lon_lin_vel, &lon_lin_vel_val_goal),

    mass(),
    wheel_radius(),
    dir(),
    dir_proxy(&dir),
    env_force(),
    needed_lon_lin_acc(&lon_lin_vel_error, &actual_act_horizon_secs_recip),
    needed_lon_force_plus_lon_env_force(&needed_lon_lin_acc, &mass),
    lon_env_force(&dir_proxy, &env_force),
    neg_lon_env_force(&lon_env_force),
    needed_lon_force(&needed_lon_force_plus_lon_env_force, &neg_lon_env_force),
    needed_motor_torque(&needed_lon_force, &wheel_radius),

    max_lim_motor_torque(&needed_motor_torque, &max_motor_torque),
    actual_motor_torque(&max_lim_motor_torque, &min_motor_torque)
{
    assert(max_motor_torque_value >= 0.0);
    assert(min_motor_torque_value <= 0.0);
}

void MotorTorqueControlFWDCar::set_fwd_car(FWDCar *fwd_car)
{
    ControlFWDCar::set_fwd_car(fwd_car);

    set_motor_torque_control(&actual_motor_torque);

    lon_lin_vel.set_parent(fwd_car->get_lon_lin_vel_variable());
    mass.set_parent(fwd_car->get_mass_variable());
    wheel_radius.set_parent(fwd_car->get_wheel_radius_variable());
    dir.set_parent(fwd_car->get_dir_variable());
    env_force.set_parent(fwd_car->get_env_force_variable());
}

}
}
}
