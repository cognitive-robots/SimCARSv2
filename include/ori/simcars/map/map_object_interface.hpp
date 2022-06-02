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
class IMapObject
{
public:
    virtual ~IMapObject() = default;

    virtual T_id get_id() const = 0;
    virtual std::shared_ptr<const IMap<T_id>> get_map() const = 0;
};

template <typename T_id>
inline bool operator ==(std::shared_ptr<const IMapObject<T_id>> lhs, std::shared_ptr<const IMapObject<T_id>> rhs)
{
    return lhs->get_id() == rhs->get_id() && lhs->get_map() == rhs->get_map();
}

}
}
}
