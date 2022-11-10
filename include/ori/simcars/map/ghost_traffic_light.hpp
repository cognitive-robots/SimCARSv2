#pragma once

#include <ori/simcars/map/traffic_light_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template<typename T_id>
class GhostTrafficLight : public ATrafficLight<T_id>
{
protected:
    mutable GhostTrafficLight<T_id> const *tether;

    GhostTrafficLight(T_id const &id, IMap<T_id> const *map) : ATrafficLight<T_id>(id, map) {}

public:
    ITrafficLight<T_id> const* get_self() const override
    {
        throw GhostTrafficLight<T_id>::GhostObjectException;
    }
    void banish() const override
    {
        tether.reset();
    }
    ITrafficLight<T_id> const* get_true_self() const noexcept override
    {
        ITrafficLight<T_id> const *true_self = this->get_map()->get_traffic_light(this->get_id());
        tether.reset();
        return true_self;
    }

    geometry::Vec const& get_position() const override
    {
        throw GhostTrafficLight<T_id>::GhostObjectException;
    }
    FP_DATA_TYPE get_orientation() const override
    {
        throw GhostTrafficLight<T_id>::GhostObjectException;
    }
    structures::IArray<ITrafficLightStateHolder::FaceColour> const* get_face_colours() const override
    {
        throw GhostTrafficLight<T_id>::GhostObjectException;
    }
    GhostTrafficLight<T_id>::FaceType get_face_type(GhostTrafficLight<T_id>::FaceColour face_colour) const override
    {
        throw GhostTrafficLight<T_id>::GhostObjectException;
    }

    static GhostTrafficLight<T_id>* spawn(T_id const &id, IMap<T_id> const *map)
    {
        GhostTrafficLight<T_id> *spawned_ghost_traffic_light = new GhostTrafficLight<T_id>(id, map);
        spawned_ghost_traffic_light->tether = spawned_ghost_traffic_light;
        return spawned_ghost_traffic_light;
    }
};

}
}
}
