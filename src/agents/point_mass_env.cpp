
#include <ori/simcars/agents/point_mass_env.hpp>

#include <ori/simcars/agents/point_mass.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

PointMassEnv::PointMassEntity::PointMassLink::PointMassLink(uint64_t id, PointMass *point_mass,
                                                            uint64_t other_id,
                                                            PointMass *other_point_mass) :
    id(id),
    point_mass(point_mass),
    other_id(other_id),
    other_point_mass(other_point_mass),

    neg_other_pos(other_point_mass->get_pos_variable()),
    pos_diff(point_mass->get_pos_variable(), &neg_other_pos),

    dist_scale_factor(1.0),
    dist(&pos_diff),
    scaled_dist(&dist, &dist_scale_factor),
    neg_scaled_dist(&scaled_dist),
    force_mag(&neg_scaled_dist),

    force_dir(&pos_diff),
    actual_avoidance_force(&force_dir, &force_mag)
{
}

causal::IEndogenousVariable<FP_DATA_TYPE>* PointMassEnv::PointMassEntity::PointMassLink::get_dist()
{
    return &dist;
}

causal::IEndogenousVariable<geometry::Vec>* PointMassEnv::PointMassEntity::PointMassLink::get_avoidance_force()
{
    return &actual_avoidance_force;
}

PointMassEnv::PointMassEntity::PointMassEntity(uint64_t id, PointMass *point_mass) :
    id(id),
    point_mass(point_mass),

    neighbour_dist_limit(2.0),
    neighbour_dist_limit_proxy(&neighbour_dist_limit),

    env_force({}),

    min_neighbour_dist({ &neighbour_dist_limit_proxy })
{
}

PointMassEnv::PointMassEntity::~PointMassEntity()
{
    structures::IArray<PointMassLink*> const *link_array = id_link_dict.get_values();
    for (size_t i = 0; i < link_array->count(); ++i)
    {
        delete (*link_array)[i];
    }
}

causal::IEndogenousVariable<geometry::Vec>* PointMassEnv::PointMassEntity::get_env_force()
{
    return &env_force;
}

causal::IEndogenousVariable<FP_DATA_TYPE>* PointMassEnv::PointMassEntity::get_min_neighbour_dist()
{
    return &min_neighbour_dist;
}

bool PointMassEnv::PointMassEntity::add_link(PointMass *other_point_mass)
{
    simcars::causal::IEndogenousVariable<uint64_t> *id_variable =
            other_point_mass->get_id_variable();

    uint64_t other_id;
    bool res = id_variable->get_value(other_id);

    if (!res || other_id == id || id_link_dict.contains(other_id))
    {
        return false;
    }
    else
    {
        PointMassLink *link = new PointMassLink(id, point_mass, other_id, other_point_mass);

        id_link_dict.update(other_id, link);

        env_force.insert(link->get_avoidance_force());
        min_neighbour_dist.insert(link->get_dist());

        return true;
    }
}

bool PointMassEnv::PointMassEntity::remove_link(PointMass *other_point_mass)
{
    simcars::causal::IEndogenousVariable<uint64_t> *id_variable =
            other_point_mass->get_id_variable();

    uint64_t other_id;
    bool res = id_variable->get_value(other_id);

    if (!res || other_id == id || !id_link_dict.contains(other_id))
    {
        return false;
    }
    else
    {
        PointMassLink *link = id_link_dict[other_id];

        env_force.erase(link->get_avoidance_force());
        min_neighbour_dist.erase(link->get_dist());

        id_link_dict.erase(other_id);

        delete link;

        return true;
    }
}

PointMassEnv::~PointMassEnv()
{
    structures::IArray<PointMassEntity*> const *entity_array = id_entity_dict.get_values();
    for (size_t i = 0; i < entity_array->count(); ++i)
    {
        delete (*entity_array)[i];
    }
}

structures::IArray<PointMass*> const* PointMassEnv::get_point_masses() const
{
    return id_point_mass_dict.get_values();
}

bool PointMassEnv::add_point_mass(PointMass *point_mass)
{
    simcars::causal::IEndogenousVariable<uint64_t> *id_variable =
            point_mass->get_id_variable();

    uint64_t id;
    bool res = id_variable->get_value(id);

    if (!res || id_entity_dict.contains(id))
    {
        return false;
    }
    else
    {
        PointMassEntity *entity = new PointMassEntity(id, point_mass);

        structures::IArray<uint64_t> const *id_array = id_entity_dict.get_keys();
        for (size_t i = 0; i < id_array->count(); ++i)
        {
            id_entity_dict[(*id_array)[i]]->add_link(point_mass);
            entity->add_link(id_point_mass_dict[(*id_array)[i]]);
        }

        id_point_mass_dict.update(id, point_mass);
        id_entity_dict.update(id, entity);

        point_mass->env_force.set_parent(entity->get_env_force());
        point_mass->env_force_buff.set_axiomatic(false);

        point_mass->min_neighbour_dist.set_parent(entity->get_min_neighbour_dist());
        point_mass->min_neighbour_dist_buff.set_axiomatic(false);

        return true;
    }
}

bool PointMassEnv::remove_point_mass(PointMass *point_mass)
{
    simcars::causal::IEndogenousVariable<uint64_t> *id_variable =
            point_mass->get_id_variable();

    uint64_t id;
    bool res = id_variable->get_value(id);

    if (!res || !id_entity_dict.contains(id))
    {
        return false;
    }
    else
    {
        PointMassEntity *entity = id_entity_dict[id];

        point_mass->env_force_buff.set_axiomatic(true);
        point_mass->env_force.set_parent(nullptr);

        point_mass->min_neighbour_dist_buff.set_axiomatic(true);
        point_mass->min_neighbour_dist.set_parent(nullptr);

        id_entity_dict.erase(id);
        id_point_mass_dict.erase(id);

        structures::IArray<uint64_t> const *id_array = id_entity_dict.get_keys();
        for (size_t i = 0; i < id_array->count(); ++i)
        {
            id_entity_dict[(*id_array)[i]]->remove_link(point_mass);
            entity->remove_link(id_point_mass_dict[(*id_array)[i]]);
        }

        delete entity;

        return true;
    }
}

}
}
}
