
#include <ori/simcars/agents/fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

FWDCar::FWDCar(FP_DATA_TYPE mass_value, FP_DATA_TYPE length_value, FP_DATA_TYPE width_value,
               FP_DATA_TYPE height_value, FP_DATA_TYPE wheel_radius_value,
               FP_DATA_TYPE axel_dist_value, FP_DATA_TYPE drag_area_value,
               FP_DATA_TYPE cornering_stiffness_value) :
    RectRigidBody(mass_value, length_value, width_value, height_value, drag_area_value),

    wheel_radius(wheel_radius_value),
    wheel_radius_proxy(&wheel_radius),
    wheel_radius_recip(&wheel_radius),

    axel_dist(axel_dist_value),
    axel_dist_proxy(&axel_dist),
    neg_axel_dist(&axel_dist),

    cornering_stiffness(cornering_stiffness_value),

    dir(&rot_buff),

    lon_lin_vel(&lin_vel_buff, &dir),
    lon_lin_vel_recip(&lon_lin_vel),

    lat_lin_vel(&lin_vel_buff, &dir),

    motor_torque(),
    motor_torque_buff(&motor_torque, nullptr, true),
    front_wheel_lon_force_mag(&wheel_radius_recip, &motor_torque_buff),

    rear_wheel_lon_force_mag(0.0),
    rear_wheel_lon_force_mag_proxy(&rear_wheel_lon_force_mag),

    front_wheel_ang_lat_lin_vel(&ang_vel_buff, &axel_dist),
    front_wheel_lat_lin_vel(&lat_lin_vel, &front_wheel_ang_lat_lin_vel),
    neg_front_wheel_slip_ang_minus_steer(&front_wheel_lat_lin_vel, &lon_lin_vel_recip),
    front_wheel_slip_ang_minus_steer(&neg_front_wheel_slip_ang_minus_steer),
    steer(),
    steer_buff(&steer, nullptr, true),
    front_wheel_slip_ang(&front_wheel_slip_ang_minus_steer, &steer_buff),
    front_wheel_lat_force_mag(&front_wheel_slip_ang, &cornering_stiffness),

    rear_wheel_ang_lat_lin_vel(&ang_vel_buff, &neg_axel_dist),
    rear_wheel_lat_lin_vel(&lat_lin_vel, &rear_wheel_ang_lat_lin_vel),
    neg_rear_wheel_slip_ang(&rear_wheel_lat_lin_vel, &lon_lin_vel_recip),
    rear_wheel_slip_ang(&neg_rear_wheel_slip_ang),
    rear_wheel_lat_force_mag(&rear_wheel_slip_ang, &cornering_stiffness),

    front_wheel_local_force(&front_wheel_lon_force_mag, &front_wheel_lat_force_mag),
    steer_mat(&steer_buff),
    steered_front_wheel_local_force(&steer_mat, &front_wheel_local_force),

    rear_wheel_local_force(&rear_wheel_lon_force_mag_proxy, &rear_wheel_lat_force_mag),

    front_wheel_torque_force_mag(&steered_front_wheel_local_force),
    front_wheel_torque(&front_wheel_torque_force_mag, &axel_dist),

    rear_wheel_torque_force_mag(&rear_wheel_local_force),
    rear_wheel_torque(&rear_wheel_torque_force_mag, &neg_axel_dist),

    combined_wheel_local_force(&steered_front_wheel_local_force, &rear_wheel_local_force),
    rot_mat(&rot_buff),
    combined_wheel_force(&rot_mat, &combined_wheel_local_force),

    combined_wheel_torque(&front_wheel_torque, &rear_wheel_torque)
{
    assert(wheel_radius_value > 0.0);
    assert(axel_dist_value > 0.0);
    assert(axel_dist_value <= length_value / 2.0);
    assert(cornering_stiffness_value > 0.0);

    RectRigidBody::other_force.set_parent(&combined_wheel_force);
    RectRigidBody::other_torque.set_parent(&combined_wheel_torque);
}

FWDCar::FWDCar(FWDCar const &fwd_car) :
    RectRigidBody(fwd_car),

    wheel_radius(fwd_car.wheel_radius),
    wheel_radius_proxy(&wheel_radius),
    wheel_radius_recip(&wheel_radius),

    axel_dist(fwd_car.axel_dist),
    axel_dist_proxy(&axel_dist),
    neg_axel_dist(&axel_dist),

    cornering_stiffness(fwd_car.cornering_stiffness),

    dir(&rot_buff),

    lon_lin_vel(&lin_vel_buff, &dir),
    lon_lin_vel_recip(&lon_lin_vel),

    lat_lin_vel(&lin_vel_buff, &dir),

    motor_torque(),
    motor_torque_buff(&motor_torque, nullptr, true),
    front_wheel_lon_force_mag(&wheel_radius_recip, &motor_torque_buff),

    rear_wheel_lon_force_mag(0.0),
    rear_wheel_lon_force_mag_proxy(&rear_wheel_lon_force_mag),

    front_wheel_ang_lat_lin_vel(&ang_vel_buff, &axel_dist),
    front_wheel_lat_lin_vel(&lat_lin_vel, &front_wheel_ang_lat_lin_vel),
    neg_front_wheel_slip_ang_minus_steer(&front_wheel_lat_lin_vel, &lon_lin_vel_recip),
    front_wheel_slip_ang_minus_steer(&neg_front_wheel_slip_ang_minus_steer),
    steer(),
    steer_buff(&steer, nullptr, true),
    front_wheel_slip_ang(&front_wheel_slip_ang_minus_steer, &steer_buff),
    front_wheel_lat_force_mag(&front_wheel_slip_ang, &cornering_stiffness),

    rear_wheel_ang_lat_lin_vel(&ang_vel_buff, &neg_axel_dist),
    rear_wheel_lat_lin_vel(&lat_lin_vel, &rear_wheel_ang_lat_lin_vel),
    neg_rear_wheel_slip_ang(&rear_wheel_lat_lin_vel, &lon_lin_vel_recip),
    rear_wheel_slip_ang(&neg_rear_wheel_slip_ang),
    rear_wheel_lat_force_mag(&rear_wheel_slip_ang, &cornering_stiffness),

    front_wheel_local_force(&front_wheel_lon_force_mag, &front_wheel_lat_force_mag),
    steer_mat(&steer_buff),
    steered_front_wheel_local_force(&steer_mat, &front_wheel_local_force),

    rear_wheel_local_force(&rear_wheel_lon_force_mag_proxy, &rear_wheel_lat_force_mag),

    front_wheel_torque_force_mag(&steered_front_wheel_local_force),
    front_wheel_torque(&front_wheel_torque_force_mag, &axel_dist),

    rear_wheel_torque_force_mag(&rear_wheel_local_force),
    rear_wheel_torque(&rear_wheel_torque_force_mag, &neg_axel_dist),

    combined_wheel_local_force(&front_wheel_local_force, &rear_wheel_local_force),
    rot_mat(&rot_buff),
    combined_wheel_force(&rot_mat, &combined_wheel_local_force),

    combined_wheel_torque(&front_wheel_torque, &rear_wheel_torque)
{
    RectRigidBody::other_force.set_parent(&combined_wheel_force);
    RectRigidBody::other_torque.set_parent(&combined_wheel_torque);
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* FWDCar::get_wheel_radius_variable()
{
    return &wheel_radius_proxy;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* FWDCar::get_axel_dist_variable()
{
    return &axel_dist_proxy;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* FWDCar::get_lon_lin_vel_variable()
{
    return &lon_lin_vel;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* FWDCar::get_lon_lin_vel_recip_variable()
{
    return &lon_lin_vel_recip;
}

simcars::causal::IEndogenousVariable<geometry::Vec>* FWDCar::get_dir_variable()
{
    return &dir;
}

}
}
}
