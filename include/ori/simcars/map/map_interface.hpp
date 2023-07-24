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

class IMap
{
public:
    virtual ~IMap() = default;

    virtual ILane const* get_lane(uint64_t id) const = 0;
    virtual structures::IArray<ILane const*>* get_lanes(structures::IArray<uint64_t> const *ids) const = 0;
    virtual structures::IArray<ILane const*>* get_encapsulating_lanes(geometry::Vec point) const = 0;
    virtual structures::IArray<ILane const*>* get_lanes_in_range(geometry::Vec point, FP_DATA_TYPE distance) const = 0;
};

}
}
}
