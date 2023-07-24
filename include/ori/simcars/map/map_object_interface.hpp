#pragma once

#include <ori/simcars/map/declarations.hpp>
#include <ori/simcars/map/map_interface.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

class IMapObject
{
public:
    enum class Type
    {
        UNKNOWN = 0,
        LANE = 1
    };

    virtual ~IMapObject() = default;

    virtual uint64_t get_id() const = 0;
    virtual Type get_type() const = 0;
    virtual IMap const* get_map() const = 0;
};

}
}
}
