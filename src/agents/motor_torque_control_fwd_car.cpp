
#include <ori/simcars/agents/motor_torque_control_fwd_car.hpp>

#include <ori/simcars/agents/fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

void MotorTorqueControlFWDCar::init_links()
{
    speed.set_parent(fwd_car->get_lon_lin_vel_variable());
    mass.set_parent(fwd_car->get_mass_variable());
    wheel_radius.set_parent(fwd_car->get_wheel_radius_variable());
    dir.set_parent(fwd_car->get_dir_variable());
    env_force.set_parent(fwd_car->get_env_force_variable());

    motor_torque.set_parent(&actual_motor_torque);
}

MotorTorqueControlFWDCar::MotorTorqueControlFWDCar(FP_DATA_TYPE max_motor_torque_value,
                                                   FP_DATA_TYPE min_motor_torque_value) :
    max_motor_torque(max_motor_torque_value),
    min_motor_torque(min_motor_torque_value),

    time_diff(&speed_time_goal),
    time_diff_secs(&time_diff),
    min_act_horizon_secs(1.0),
    actual_act_horizon_secs(&time_diff_secs, &min_act_horizon_secs),
    actual_act_horizon_secs_recip(&actual_act_horizon_secs),

    speed(),
    neg_speed(&speed),
    speed_diff(&neg_speed, &speed_val_goal),

    mass(),
    wheel_radius(),
    dir(),
    dir_proxy(&dir),
    env_force(),
    needed_acc(&speed_diff, &actual_act_horizon_secs_recip),
    needed_force_plus_lon_env_force(&needed_acc, &mass),
    lon_env_force(&dir_proxy, &env_force),
    neg_lon_env_force(&lon_env_force),
    needed_force(&needed_force_plus_lon_env_force, &neg_lon_env_force),
    needed_motor_torque(&needed_force, &wheel_radius),

    max_lim_motor_torque(&needed_motor_torque, &max_motor_torque),
    actual_motor_torque(&max_lim_motor_torque, &min_motor_torque)
{
    assert(max_motor_torque_value >= 0.0);
    assert(min_motor_torque_value <= 0.0);
}

MotorTorqueControlFWDCar::MotorTorqueControlFWDCar(MotorTorqueControlFWDCar const &control_fwd_car) :
    max_motor_torque(control_fwd_car.max_motor_torque),
    min_motor_torque(control_fwd_car.min_motor_torque),

    time_diff(&speed_time_goal),
    time_diff_secs(&time_diff),
    min_act_horizon_secs(1.0),
    actual_act_horizon_secs(&time_diff_secs, &min_act_horizon_secs),
    actual_act_horizon_secs_recip(&actual_act_horizon_secs),

    speed(),
    neg_speed(&speed),
    speed_diff(&neg_speed, &speed_val_goal),

    mass(),
    wheel_radius(),
    dir(),
    dir_proxy(&dir),
    env_force(),
    needed_acc(&speed_diff, &actual_act_horizon_secs_recip),
    needed_force_plus_lon_env_force(&needed_acc, &mass),
    lon_env_force(&dir_proxy, &env_force),
    neg_lon_env_force(&lon_env_force),
    needed_force(&needed_force_plus_lon_env_force, &neg_lon_env_force),
    needed_motor_torque(&needed_force, &wheel_radius),

    max_lim_motor_torque(&needed_motor_torque, &max_motor_torque),
    actual_motor_torque(&max_lim_motor_torque, &min_motor_torque)
{
}

}
}
}
