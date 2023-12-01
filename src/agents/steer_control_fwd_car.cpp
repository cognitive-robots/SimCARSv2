
#include <ori/simcars/agents/steer_control_fwd_car.hpp>

#include <ori/simcars/agents/fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

void SteerControlFWDCar::init_links()
{
    pos.set_parent(fwd_car->get_pos_variable());
    lin_vel.set_parent(fwd_car->get_lin_vel_variable());
    axel_dist.set_parent(fwd_car->get_axel_dist_variable());
    lon_lin_vel_recip.set_parent(fwd_car->get_lon_lin_vel_recip_variable());

    steer.set_parent(&actual_steer);
}

SteerControlFWDCar::SteerControlFWDCar(map::IMap const *map, FP_DATA_TYPE max_abs_steer_value) :
    max_steer(max_abs_steer_value),
    min_steer(&max_steer),

    lane_val_goal_proxy(&lane_val_goal),

    time_error(&lane_time_goal),
    time_error_secs(&time_error),
    min_act_horizon_secs(1.0),
    actual_act_horizon_secs(&time_error_secs, &min_act_horizon_secs),
    actual_act_horizon_secs_recip(&actual_act_horizon_secs),

    pos(),
    pos_proxy(&pos),
    lin_vel(),
    lin_vel_proxy(&lin_vel),
    act_horizon_expected_pos_diff(&lin_vel_proxy, &actual_act_horizon_secs),
    act_horizon_expected_pos(&pos_proxy, &act_horizon_expected_pos_diff),
    act_horizon_target_pos(&lane_val_goal_proxy, &act_horizon_expected_pos, map),
    neg_pos(&pos),
    act_horizon_target_pos_diff(&act_horizon_target_pos, &neg_pos),

    act_horizon_ang_sin(&act_horizon_expected_pos_diff, &act_horizon_target_pos_diff),
    act_horizon_ang_cos(&act_horizon_expected_pos_diff, &act_horizon_target_pos_diff),
    act_horizon_ang_cos_recip(&act_horizon_ang_cos),
    act_horizon_ang(&act_horizon_ang_sin, &act_horizon_ang_cos_recip),

    needed_ang_vel(&act_horizon_ang, &actual_act_horizon_secs_recip),

    axel_dist(),
    double_scale_factor(2.0),
    double_scale_factor_proxy(&double_scale_factor),
    double_axel_dist(&double_scale_factor_proxy, &axel_dist),

    lon_lin_vel_recip(),
    needed_ang_vel_double_axel_dist_prod(&needed_ang_vel, &double_axel_dist),
    needed_steer(&needed_ang_vel_double_axel_dist_prod, &lon_lin_vel_recip),

    max_lim_steer(&needed_steer, &max_steer),
    actual_steer(&max_lim_steer, &min_steer)
{
    assert(max_abs_steer_value >= 0.0);
}

}
}
}
