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
    mutable std::shared_ptr<const GhostTrafficLight<T_id>> tether;

    GhostTrafficLight(const T_id& id, std::shared_ptr<const IMap<T_id>> map) : ATrafficLight<T_id>(id, map) {}

public:
    std::shared_ptr<const ITrafficLight<T_id>> get_self() const override
    {
        throw GhostTrafficLight<T_id>::GhostObjectException;
    }
    void banish() const override
    {
        tether.reset();
    }
    std::shared_ptr<const ITrafficLight<T_id>> get_true_self() const noexcept override
    {
        std::shared_ptr<const ITrafficLight<T_id>> true_self = this->get_map()->get_traffic_light(this->get_id());
        tether.reset();
        return true_self;
    }

    const geometry::Vec& get_position() const override
    {
        throw GhostTrafficLight<T_id>::GhostObjectException;
    }
    FP_DATA_TYPE get_orientation() const override
    {
        throw GhostTrafficLight<T_id>::GhostObjectException;
    }
    std::shared_ptr<const structures::IArray<ITrafficLightStateHolder::FaceColour>> get_face_colours() const override
    {
        throw GhostTrafficLight<T_id>::GhostObjectException;
    }
    GhostTrafficLight<T_id>::FaceType get_face_type(GhostTrafficLight<T_id>::FaceColour face_colour) const override
    {
        throw GhostTrafficLight<T_id>::GhostObjectException;
    }

    static std::shared_ptr<GhostTrafficLight<T_id>> spawn(const T_id& id, std::shared_ptr<const IMap<T_id>> map)
    {
        std::shared_ptr<GhostTrafficLight<T_id>> spawned_ghost_traffic_light(new GhostTrafficLight<T_id>(id, map));
        spawned_ghost_traffic_light->tether = spawned_ghost_traffic_light;
        return spawned_ghost_traffic_light;
    }
};

}
}
}
