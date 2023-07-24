#pragma once

#include <ori/simcars/map/map_object_interface.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

class AMapObject : public virtual IMapObject
{
    uint64_t const id;
    IMap const* const map;

public:
    AMapObject(uint64_t id, IMap const *map);

    uint64_t get_id() const override;
    IMap const* get_map() const override;
};

}
}
}
