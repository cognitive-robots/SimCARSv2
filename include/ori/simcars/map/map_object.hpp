#pragma once

#include <ori/simcars/map/declarations.hpp>
#include <ori/simcars/map/map_interface.hpp>

#include <memory>

namespace ori
{
namespace simcars
{
namespace map
{

template<typename T_id>
class MapObject
{
    const T_id id;
    const std::weak_ptr<const IMap<T_id>> map;

public:
    MapObject(const T_id& id, std::shared_ptr<const IMap<T_id>> map) : id(id), map(map) {}
    virtual ~MapObject() = default;

    bool operator ==(const MapObject<T_id>& map_object) const
    {
        return this->id == map_object.id && this->map.lock() == map_object.map.lock();
    }
    T_id get_id() const
    {
        return id;
    }
    std::shared_ptr<const IMap<T_id>> get_map() const
    {
        return map.lock();
    }
};

}
}
}
