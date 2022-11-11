#pragma once

#include <ori/simcars/map/map_object_interface.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template<typename T_id>
class MapObject : public virtual IMapObject<T_id>
{
    T_id const id;
    IMap<T_id> const* const map;

public:
    MapObject(T_id const &id, IMap<T_id> const *map) : id(id), map(map) {}

    T_id get_id() const override
    {
        return id;
    }
    IMap<T_id> const* get_map() const override
    {
        return map;
    }
};

}
}
}
