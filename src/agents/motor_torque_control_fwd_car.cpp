
#include <ori/simcars/agents/motor_torque_control_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

MotorTorqueControlFWDCar::MotorTorqueControlFWDCar(FP_DATA_TYPE mass_value,
                                                   FP_DATA_TYPE length_value,
                                                   FP_DATA_TYPE width_value,
                                                   FP_DATA_TYPE height_value,
                                                   FP_DATA_TYPE wheel_radius_value,
                                                   FP_DATA_TYPE axel_dist_value,
                                                   FP_DATA_TYPE max_motor_torque_value,
                                                   FP_DATA_TYPE min_motor_torque_value,
                                                   FP_DATA_TYPE drag_area_value,
                                                   FP_DATA_TYPE cornering_stiffness_value) :
    FWDCar(mass_value, length_value, width_value, height_value, wheel_radius_value,
           axel_dist_value, drag_area_value, cornering_stiffness_value),

    max_motor_torque(max_motor_torque_value),
    min_motor_torque(min_motor_torque_value),

    lon_lin_vel_time_goal(),
    lon_lin_vel_val_goal(),

    time_error(&lon_lin_vel_time_goal),
    time_error_secs(&time_error),
    min_act_horizon_secs(1.0),
    actual_act_horizon_secs(&time_error_secs, &min_act_horizon_secs),
    actual_act_horizon_secs_recip(&actual_act_horizon_secs),

    neg_lon_lin_vel(&lon_lin_vel),
    lon_lin_vel_error(&neg_lon_lin_vel, &lon_lin_vel_val_goal),

    needed_lon_lin_acc(&lon_lin_vel_error, &actual_act_horizon_secs_recip),
    needed_lon_force_plus_lon_env_force(&needed_lon_lin_acc, &mass),
    lon_env_force(&dir, &env_force),
    neg_lon_env_force(&lon_env_force),
    needed_lon_force(&needed_lon_force_plus_lon_env_force, &neg_lon_env_force),
    needed_motor_torque(&needed_lon_force, &wheel_radius),

    max_lim_motor_torque(&needed_motor_torque, &max_motor_torque),
    actual_motor_torque(&max_lim_motor_torque, &min_motor_torque)
{
    assert(max_motor_torque_value >= 0.0);
    assert(min_motor_torque_value <= 0.0);

    FWDCar::motor_torque.set_parent(&actual_motor_torque);
}

}
}
}
