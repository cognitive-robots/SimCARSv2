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
public:
    GhostTrafficLight(T_id const &id, IMap<T_id> const *map) : ATrafficLight<T_id>(id, map) {}

    ITrafficLight<T_id> const* get_self() const override
    {
        throw GhostTrafficLight<T_id>::GhostObjectException;
    }
    ITrafficLight<T_id> const* get_true_self() const noexcept override
    {
        ITrafficLight<T_id> const *true_self = this->get_map()->get_traffic_light(this->get_id());
        delete this;
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
};

}
}
}
