#pragma once

#include <ori/simcars/map/declarations.hpp>
#include <ori/simcars/map/ped_map_interface.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

class IPedMapObject
{
public:
    virtual ~IPedMapObject() = default;

    virtual uint64_t get_id() const = 0;
    virtual IPedMap const* get_map() const = 0;
};

}
}
}
