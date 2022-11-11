#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/temporal/stateful_interface.hpp>
#include <ori/simcars/map/soul_interface.hpp>
#include <ori/simcars/map/declarations.hpp>
#include <ori/simcars/map/map_object_interface.hpp>
#include <ori/simcars/map/traffic_light_state_holder_interface.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class ITrafficLight : public virtual IMapObject<T_id>, public ITrafficLightStateHolder, public temporal::IStateful<ITrafficLightStateHolder>, public virtual ISoul<ITrafficLight<T_id>>
{
public:
    virtual geometry::Vec const& get_position() const = 0;
    virtual FP_DATA_TYPE get_orientation() const = 0;
    virtual structures::IArray<ITrafficLightStateHolder::FaceColour> const* get_face_colours() const = 0;
    virtual ITrafficLight<T_id>::FaceType get_face_type(ITrafficLight<T_id>::FaceColour face_colour) const = 0;
};

}
}
}
