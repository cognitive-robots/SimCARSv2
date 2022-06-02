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
    const T_id id;
    const std::weak_ptr<const IMap<T_id>> map;

public:
    MapObject(const T_id& id, std::shared_ptr<const IMap<T_id>> map) : id(id), map(map) {}

    T_id get_id() const override
    {
        return id;
    }
    std::shared_ptr<const IMap<T_id>> get_map() const override
    {
        return map.lock();
    }
};

}
}
}
