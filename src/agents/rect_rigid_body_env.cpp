
#include <ori/simcars/agents/rect_rigid_body_env.hpp>

#include <ori/simcars/agents/rect_rigid_body.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

RectRigidBodyEnv::Entity::Link::Link(RectRigidBody const *rigid_body,
                                     RectRigidBody const *other_rigid_body) :
    mass_sum(rigid_body->get_mass_variable(), other_rigid_body->get_mass_variable()),
    mass_sum_recip(&mass_sum),

    coll(rigid_body->get_rect_variable(), other_rigid_body->get_rect_variable()),

    coll_contact(rigid_body->get_rect_variable(), other_rigid_body->get_rect_variable()),
    coll_contact_pos(&coll_contact),
    coll_contact_dir(&coll_contact),

    ali_lin_vel_mag(&coll_contact_dir, rigid_body->get_lin_vel_variable()),
    other_ali_lin_vel_mag(&coll_contact_dir, other_rigid_body->get_lin_vel_variable()),

    ali_lin_mom_mag(rigid_body->get_mass_variable(), &ali_lin_vel_mag),
    other_ali_lin_mom_mag(other_rigid_body->get_mass_variable(), &other_ali_lin_vel_mag),
    ali_lin_mom_mag_sum(&ali_lin_mom_mag, &other_ali_lin_mom_mag),
    coll_lin_vel_mag(&mass_sum_recip, &ali_lin_mom_mag_sum),

    neg_ali_lin_vel_mag(&ali_lin_vel_mag),

    coll_lin_vel_mag_diff(&coll_lin_vel_mag, &neg_ali_lin_vel_mag),
    coll_lin_acc_mag(&coll_lin_vel_mag_diff),
    coll_force_mag(rigid_body->get_mass_variable(), &coll_lin_acc_mag),
    coll_force(&coll_contact_dir, &coll_force_mag),

    no_coll_force(geometry::Vec::Zero()),
    no_coll_force_proxy(&no_coll_force),

    actual_coll_force(&coll_force, &no_coll_force_proxy, &coll),

    neg_pos(rigid_body->get_pos_variable()),
    coll_contact_rel_pos(&coll_contact_pos, &neg_pos),

    coll_torque(&coll_force, &coll_contact_rel_pos),

    no_coll_torque(0.0),
    no_coll_torque_proxy(&no_coll_torque),

    actual_coll_torque(&coll_torque, &no_coll_torque_proxy, &coll)
{
}

causal::IEndogenousVariable<geometry::Vec> const* RectRigidBodyEnv::Entity::Link::get_coll_force() const
{
    return &actual_coll_force;
}

causal::IEndogenousVariable<FP_DATA_TYPE> const* RectRigidBodyEnv::Entity::Link::get_coll_torque() const
{
    return &actual_coll_torque;
}

RectRigidBodyEnv::Entity::Entity(RectRigidBody const *rigid_body) :
    half_scale_factor(0.5),
    half_scale_factor_proxy(&half_scale_factor),

    air_mass_density(1.2578),
    drag_scaled_air_mass_density(&half_scale_factor_proxy, &air_mass_density),

    lin_vel_dir(rigid_body->get_lin_vel_variable()),
    drag_force_dir(&lin_vel_dir),

    lin_spd_squared(rigid_body->get_lin_vel_variable(), rigid_body->get_lin_vel_variable()),
    dynamic_pressure(&drag_scaled_air_mass_density, &lin_spd_squared),
    drag_force_mag(&dynamic_pressure, rigid_body->get_drag_area_variable()),

    drag_force(&drag_force_dir, &drag_force_mag),
    drag_torque(0.0),
    drag_torque_proxy(&drag_torque),

    env_force({ &drag_force }),
    env_torque({ &drag_torque_proxy })
{
}

RectRigidBodyEnv::Entity::~Entity()
{
    structures::IArray<Link*> const *link_array = other_rigid_body_link_dict.get_values();
    for (size_t i = 0; i < link_array->count(); ++i)
    {
        delete (*link_array)[i];
    }
}

causal::IEndogenousVariable<geometry::Vec> const* RectRigidBodyEnv::Entity::get_env_force() const
{
    return &env_force;
}

causal::IEndogenousVariable<FP_DATA_TYPE> const* RectRigidBodyEnv::Entity::get_env_torque() const
{
    return &env_torque;
}

bool RectRigidBodyEnv::Entity::add_link(RectRigidBody const *other_rigid_body)
{
    if (other_rigid_body == rigid_body || other_rigid_body_link_dict.contains(other_rigid_body))
    {
        return false;
    }
    else
    {
        Link *link = new Link(rigid_body, other_rigid_body);

        other_rigid_body_link_dict.update(other_rigid_body, link);

        env_force.insert(link->get_coll_force());
        env_torque.insert(link->get_coll_torque());

        return true;
    }
}

bool RectRigidBodyEnv::Entity::remove_link(RectRigidBody const *other_rigid_body)
{
    if (other_rigid_body == rigid_body || !other_rigid_body_link_dict.contains(other_rigid_body))
    {
        return false;
    }
    else
    {
        Link *link = other_rigid_body_link_dict[other_rigid_body];

        env_force.erase(link->get_coll_force());
        env_torque.erase(link->get_coll_torque());

        other_rigid_body_link_dict.erase(other_rigid_body);

        return true;
    }
}

RectRigidBodyEnv::~RectRigidBodyEnv()
{
    structures::IArray<Entity*> const *entity_array = rigid_body_entity_dict.get_values();
    for (size_t i = 0; i < entity_array->count(); ++i)
    {
        delete (*entity_array)[i];
    }
}

structures::IArray<RectRigidBody const*> const* RectRigidBodyEnv::get_rigid_bodies() const
{
    return rigid_body_entity_dict.get_keys();
}

bool RectRigidBodyEnv::add_rigid_body(RectRigidBody *rigid_body)
{
    if (rigid_body_entity_dict.contains(rigid_body))
    {
        return false;
    }
    else
    {
        structures::IArray<Entity*> const *entity_array = rigid_body_entity_dict.get_values();
        for (size_t i = 0; i < entity_array->count(); ++i)
        {
            (*entity_array)[i]->add_link(rigid_body);
        }

        Entity *entity = new Entity(rigid_body);

        rigid_body_entity_dict.update(rigid_body, entity);

        rigid_body->env_force.set_parent(entity->get_env_force());
        rigid_body->env_torque.set_parent(entity->get_env_torque());

        return true;
    }
}

bool RectRigidBodyEnv::remove_rigid_body(RectRigidBody *rigid_body)
{
    if (!rigid_body_entity_dict.contains(rigid_body))
    {
        return false;
    }
    else
    {
        rigid_body->env_force.set_parent(nullptr);
        rigid_body->env_torque.set_parent(nullptr);

        Entity *entity = rigid_body_entity_dict[rigid_body];

        rigid_body_entity_dict.erase(rigid_body);

        delete entity;

        structures::IArray<Entity*> const *entity_array = rigid_body_entity_dict.get_values();
        for (size_t i = 0; i < entity_array->count(); ++i)
        {
            (*entity_array)[i]->remove_link(rigid_body);
        }

        return true;
    }
}

}
}
}
