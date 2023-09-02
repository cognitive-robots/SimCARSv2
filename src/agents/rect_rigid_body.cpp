
#include <ori/simcars/agents/rect_rigid_body.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

RectRigidBody::RectRigidBody(FP_DATA_TYPE mass_value, FP_DATA_TYPE length_value,
                             FP_DATA_TYPE width_value, FP_DATA_TYPE height_value,
                             FP_DATA_TYPE drag_area_value) :
    mass(mass_value),
    mass_proxy(&mass),
    mass_recip(&mass),

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

    env_force(),
    env_force_proxy(&env_force),
    other_force(),
    total_force(&env_force_proxy, &other_force),

    env_torque(),
    env_torque_proxy(&env_torque),
    other_torque(),
    total_torque(&env_torque_proxy, &other_torque),

    lin_acc(&total_force, &mass_recip),
    lin_acc_buff(&lin_acc),
    prev_lin_acc(&lin_acc_buff),

    ang_acc(&total_torque, &moi_recip),
    ang_acc_buff(&ang_acc),
    prev_ang_acc(&ang_acc_buff),

    lin_vel_diff(&prev_lin_acc),
    prev_lin_vel(&lin_vel_buff),
    lin_vel(&prev_lin_vel, &lin_vel_diff),
    lin_vel_buff(&lin_vel),
    lin_vel_mean(&lin_vel_buff, &prev_lin_vel),

    ang_vel_diff(&prev_ang_acc),
    prev_ang_vel(&ang_vel_buff),
    ang_vel(&prev_ang_vel, &ang_vel_diff),
    ang_vel_buff(&ang_vel),
    ang_vel_mean(&ang_vel_buff, &prev_ang_vel),

    pos_diff(&lin_vel_mean),
    prev_pos(&pos_buff),
    pos(&prev_pos, &pos_diff),
    pos_buff(&pos),

    rot_diff(&ang_vel_mean),
    prev_rot(&rot_buff),
    rot(&prev_rot, &rot_diff),
    rot_buff(&rot),

    rect(&pos_buff, &rot_buff, &length_proxy, &width_proxy)
{
    assert(mass_value > 0.0);
    assert(length_value > 0.0);
    assert(width_value > 0.0);
    assert(height_value > 0.0);
    assert(drag_area_value > 0.0);
}

causal::IEndogenousVariable<FP_DATA_TYPE> const* RectRigidBody::get_mass_variable() const
{
    return &mass_proxy;
}

causal::IEndogenousVariable<FP_DATA_TYPE> const* RectRigidBody::get_length_variable() const
{
    return &length_proxy;
}

causal::IEndogenousVariable<FP_DATA_TYPE> const* RectRigidBody::get_width_variable() const
{
    return &width_proxy;
}

causal::IEndogenousVariable<FP_DATA_TYPE> const* RectRigidBody::get_height_variable() const
{
    return &height_proxy;
}

causal::IEndogenousVariable<FP_DATA_TYPE> const* RectRigidBody::get_drag_area_variable() const
{
    return &drag_area_proxy;
}

causal::IEndogenousVariable<geometry::Vec> const* RectRigidBody::get_lin_vel_variable() const
{
    return &lin_vel_buff;
}

causal::IEndogenousVariable<geometry::Vec> const* RectRigidBody::get_pos_variable() const
{
    return &pos_buff;
}

causal::IEndogenousVariable<geometry::ORect> const* RectRigidBody::get_rect_variable() const
{
    return &rect;
}

}
}
}
