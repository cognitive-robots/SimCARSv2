#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/map/declarations.hpp>

#include <exception>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class IMap
{
public:
    class ObjectNotFound : public std::exception
    {
    public:
        using std::exception::exception;

        char const* what() const noexcept override
        {
            return "Could not find a map object when a lookup was performed";
        }
    };

    virtual ~IMap() = default;

    virtual ILane<T_id> const* get_lane(T_id id) const = 0;
    virtual ILaneArray<T_id> const* get_encapsulating_lanes(geometry::Vec point) const = 0;
    virtual ILaneArray<T_id> const* get_lanes(structures::IArray<T_id> const *ids) const = 0;
    virtual ILaneArray<T_id> const* get_lanes_in_range(geometry::Vec point, FP_DATA_TYPE distance) const = 0;
    virtual ITrafficLight<T_id> const* get_traffic_light(T_id id) const = 0;
    virtual ITrafficLightArray<T_id> const* get_traffic_lights(structures::IArray<T_id> const *ids) const = 0;
    virtual ITrafficLightArray<T_id> const* get_traffic_lights_in_range(geometry::Vec point, FP_DATA_TYPE distance) const = 0;
    virtual void register_stray_ghost(IMapObject<T_id> const *ghost) const = 0;
    virtual void unregister_stray_ghost(IMapObject<T_id> const *ghost) const = 0;
};

}
}
}
