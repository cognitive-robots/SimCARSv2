
#include <ori/simcars/agents/rect_rigid_body_env.hpp>

#include <ori/simcars/agents/rect_rigid_body.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

RectRigidBodyEnv::Entity::Link::Link(uint64_t id, RectRigidBody *rigid_body, uint64_t other_id,
                                     RectRigidBody *other_rigid_body) :
    id(id),
    rigid_body(rigid_body),
    other_id(other_id),
    other_rigid_body(other_rigid_body),

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

    actual_coll_torque(&coll_torque, &no_coll_torque_proxy, &coll),

    dist_headway(rigid_body->get_rect_variable(), other_rigid_body->get_rect_variable())
{
}

causal::IEndogenousVariable<geometry::Vec>* RectRigidBodyEnv::Entity::Link::get_coll_force()
{
    return &actual_coll_force;
}

causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBodyEnv::Entity::Link::get_coll_torque()
{
    return &actual_coll_torque;
}

causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBodyEnv::Entity::Link::get_dist_headway()
{
    return &dist_headway;
}

RectRigidBodyEnv::Entity::Entity(uint64_t id, RectRigidBody *rigid_body) :
    id(id),
    rigid_body(rigid_body),

    half_scale_factor(0.5),
    half_scale_factor_proxy(&half_scale_factor),

    dist_headway_limit(100.0),
    dist_headway_limit_proxy(&dist_headway_limit),

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
    env_torque({ &drag_torque_proxy }),

    min_dist_headway({ &dist_headway_limit_proxy })
{
}

RectRigidBodyEnv::Entity::~Entity()
{
    structures::IArray<Link*> const *link_array = id_link_dict.get_values();
    for (size_t i = 0; i < link_array->count(); ++i)
    {
        delete (*link_array)[i];
    }
}

causal::IEndogenousVariable<geometry::Vec>* RectRigidBodyEnv::Entity::get_env_force()
{
    return &env_force;
}

causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBodyEnv::Entity::get_env_torque()
{
    return &env_torque;
}

causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBodyEnv::Entity::get_dist_headway()
{
    return &min_dist_headway;
}

bool RectRigidBodyEnv::Entity::add_link(RectRigidBody *other_rigid_body)
{
    simcars::causal::IEndogenousVariable<uint64_t> *id_variable =
            other_rigid_body->get_id_variable();

    uint64_t other_id;
    bool res = id_variable->get_value(other_id);

    if (!res || other_id == id || id_link_dict.contains(other_id))
    {
        return false;
    }
    else
    {
        Link *link = new Link(id, rigid_body, other_id, other_rigid_body);

        id_link_dict.update(other_id, link);

        env_force.insert(link->get_coll_force());
        env_torque.insert(link->get_coll_torque());
        min_dist_headway.insert(link->get_dist_headway());

        return true;
    }
}

bool RectRigidBodyEnv::Entity::remove_link(RectRigidBody *other_rigid_body)
{
    simcars::causal::IEndogenousVariable<uint64_t> *id_variable =
            other_rigid_body->get_id_variable();

    uint64_t other_id;
    bool res = id_variable->get_value(other_id);

    if (!res || other_id == id || !id_link_dict.contains(other_id))
    {
        return false;
    }
    else
    {
        Link *link = id_link_dict[other_id];

        env_force.erase(link->get_coll_force());
        env_torque.erase(link->get_coll_torque());
        min_dist_headway.erase(link->get_dist_headway());

        id_link_dict.erase(other_id);

        delete link;

        return true;
    }
}

RectRigidBodyEnv::~RectRigidBodyEnv()
{
    structures::IArray<Entity*> const *entity_array = id_entity_dict.get_values();
    for (size_t i = 0; i < entity_array->count(); ++i)
    {
        delete (*entity_array)[i];
    }
}

structures::IArray<RectRigidBody*> const* RectRigidBodyEnv::get_rigid_bodies() const
{
    return id_rigid_body_dict.get_values();
}

bool RectRigidBodyEnv::add_rigid_body(RectRigidBody *rigid_body)
{
    simcars::causal::IEndogenousVariable<uint64_t> *id_variable =
            rigid_body->get_id_variable();

    uint64_t id;
    bool res = id_variable->get_value(id);

    if (!res || id_entity_dict.contains(id))
    {
        return false;
    }
    else
    {
        Entity *entity = new Entity(id, rigid_body);

        structures::IArray<uint64_t> const *id_array = id_entity_dict.get_keys();
        for (size_t i = 0; i < id_array->count(); ++i)
        {
            id_entity_dict[(*id_array)[i]]->add_link(rigid_body);
            entity->add_link(id_rigid_body_dict[(*id_array)[i]]);
        }

        id_rigid_body_dict.update(id, rigid_body);
        id_entity_dict.update(id, entity);

        rigid_body->env_force.set_parent(entity->get_env_force());
        rigid_body->env_force_buff.set_axiomatic(false);

        rigid_body->env_torque.set_parent(entity->get_env_torque());
        rigid_body->env_torque_buff.set_axiomatic(false);

        rigid_body->dist_headway.set_parent(entity->get_dist_headway());
        rigid_body->dist_headway_buff.set_axiomatic(false);

        return true;
    }
}

bool RectRigidBodyEnv::remove_rigid_body(RectRigidBody *rigid_body)
{
    simcars::causal::IEndogenousVariable<uint64_t> *id_variable =
            rigid_body->get_id_variable();

    uint64_t id;
    bool res = id_variable->get_value(id);

    if (!res || !id_entity_dict.contains(id))
    {
        return false;
    }
    else
    {
        Entity *entity = id_entity_dict[id];

        rigid_body->env_force_buff.set_axiomatic(true);
        rigid_body->env_force.set_parent(nullptr);

        rigid_body->env_torque_buff.set_axiomatic(true);
        rigid_body->env_torque.set_parent(nullptr);

        rigid_body->dist_headway_buff.set_axiomatic(true);
        rigid_body->dist_headway.set_parent(nullptr);

        id_entity_dict.erase(id);
        id_rigid_body_dict.erase(id);

        structures::IArray<uint64_t> const *id_array = id_entity_dict.get_keys();
        for (size_t i = 0; i < id_array->count(); ++i)
        {
            id_entity_dict[(*id_array)[i]]->remove_link(rigid_body);
            entity->remove_link(id_rigid_body_dict[(*id_array)[i]]);
        }

        delete entity;

        return true;
    }
}

}
}
}
