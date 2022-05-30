#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/map/declarations.hpp>

#include <memory>
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

        const char* what() const noexcept override
        {
            return "Could not find a map object when a lookup was performed";
        }
    };

    virtual ~IMap() = default;

    virtual void save(const std::string& output_file_path_str) const = 0;
    virtual std::shared_ptr<const ILane<T_id>> get_lane(T_id id) const = 0;
    virtual std::shared_ptr<const ILane<T_id>> get_encapsulating_lane(geometry::Vec point) const = 0;
    virtual std::shared_ptr<const ILaneArray<T_id>> get_lanes(std::shared_ptr<const structures::IArray<T_id>> ids) const = 0;
    virtual std::shared_ptr<const ILaneArray<T_id>> get_lanes_in_range(geometry::Vec point, FP_DATA_TYPE distance) const = 0;
    virtual std::shared_ptr<const ITrafficLight<T_id>> get_traffic_light(T_id id) const = 0;
    virtual std::shared_ptr<const ITrafficLightArray<T_id>> get_traffic_lights(std::shared_ptr<const structures::IArray<T_id>> ids) const = 0;
    virtual std::shared_ptr<const ITrafficLightArray<T_id>> get_traffic_lights_in_range(geometry::Vec point, FP_DATA_TYPE distance) const = 0;
};

}
}
}
