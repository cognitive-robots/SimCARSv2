
#include <ori/simcars/agents/rect_rigid_body.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

RectRigidBody::RectRigidBody(uint64_t id_value, FP_DATA_TYPE mass_value, FP_DATA_TYPE length_value,
                             FP_DATA_TYPE width_value, FP_DATA_TYPE height_value,
                             FP_DATA_TYPE drag_area_value) :
    PointMass(id_value, mass_value),

    length(length_value),
    length_proxy(&length),
    length_squared(&length_proxy, &length),

    width(width_value),
    width_proxy(&width),
    width_squared(&width_proxy, &width),

    height(height_value),
    height_proxy(&height),

    drag_area(drag_area_value),
    drag_area_proxy(&drag_area),

    moi_scale_factor(0.0833),
    moi_scaled_mass(&mass_proxy, &moi_scale_factor),

    span_squared(&length_squared, &width_squared),

    moi(&moi_scaled_mass, &span_squared),
    moi_recip(&moi),

    dist_headway(100.0),
    dist_headway_buff(&dist_headway, nullptr, true),

    env_torque(),
    env_torque_buff(&env_torque, nullptr, true),
    other_torque(),
    other_torque_buff(&other_torque, nullptr, true),
    total_torque(&env_torque_buff, &other_torque_buff),

    ang_acc(&total_torque, &moi_recip),
    ang_acc_buff(&ang_acc, nullptr, false),
    prev_ang_acc(&ang_acc_buff),

    ang_vel_diff(&prev_ang_acc),
    prev_ang_vel(&ang_vel_buff),
    ang_vel(&prev_ang_vel, &ang_vel_diff),
    ang_vel_buff(&ang_vel, nullptr, false),

    rot_diff(&ang_vel_buff),
    prev_rot(&rot_buff),
    rot(&prev_rot, &rot_diff),
    rot_buff(&rot, nullptr, true),

    rect(&pos_buff, &rot_buff, &length_proxy, &width_proxy)
{
    assert(length_value > 0.0);
    assert(width_value > 0.0);
    assert(height_value > 0.0);
    assert(drag_area_value > 0.0);
}

RectRigidBody::RectRigidBody(RectRigidBody const &rect_rigid_body) :
    PointMass(rect_rigid_body),

    length(rect_rigid_body.length),
    length_proxy(&length),
    length_squared(&length_proxy, &length),

    width(rect_rigid_body.width),
    width_proxy(&width),
    width_squared(&width_proxy, &width),

    height(rect_rigid_body.height),
    height_proxy(&height),

    drag_area(rect_rigid_body.drag_area),
    drag_area_proxy(&drag_area),

    moi_scale_factor(0.0833),
    moi_scaled_mass(&mass_proxy, &moi_scale_factor),

    span_squared(&length_squared, &width_squared),

    moi(&moi_scaled_mass, &span_squared),
    moi_recip(&moi),

    dist_headway(100.0),
    dist_headway_buff(&dist_headway, nullptr, true),

    env_torque(),
    env_torque_buff(&env_torque, nullptr, true),
    other_torque(),
    other_torque_buff(&other_torque, nullptr, true),
    total_torque(&env_torque_buff, &other_torque_buff),

    ang_acc(&total_torque, &moi_recip),
    ang_acc_buff(&ang_acc, nullptr, false),
    prev_ang_acc(&ang_acc_buff),

    ang_vel_diff(&prev_ang_acc),
    prev_ang_vel(&ang_vel_buff),
    ang_vel(&prev_ang_vel, &ang_vel_diff),
    ang_vel_buff(&ang_vel, nullptr, false),

    rot_diff(&ang_vel_buff),
    prev_rot(&rot_buff),
    rot(&prev_rot, &rot_diff),
    rot_buff(&rot, nullptr, true),

    rect(&pos_buff, &rot_buff, &length_proxy, &width_proxy)
{}

temporal::Time RectRigidBody::get_min_time() const
{
    return std::max(PointMass::get_min_time(), rot_buff.get_min_time());
}

temporal::Time RectRigidBody::get_max_time() const
{
    return std::min(PointMass::get_max_time(), rot_buff.get_max_time());
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBody::get_length_variable()
{
    return &length_proxy;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBody::get_width_variable()
{
    return &width_proxy;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBody::get_height_variable()
{
    return &height_proxy;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBody::get_drag_area_variable()
{
    return &drag_area_proxy;
}

simcars::causal::IEndogenousVariable<geometry::ORect>* RectRigidBody::get_rect_variable()
{
    return &rect;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBody::get_dist_headway_variable()
{
    return &dist_headway_buff;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBody::get_env_torque_variable()
{
    return &env_torque_buff;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBody::get_other_torque_variable()
{
    return &other_torque_buff;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBody::get_ang_acc_variable()
{
    return &ang_acc_buff;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBody::get_ang_vel_variable()
{
    return &ang_vel_buff;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBody::get_rot_variable()
{
    return &rot_buff;
}

}
}
}
