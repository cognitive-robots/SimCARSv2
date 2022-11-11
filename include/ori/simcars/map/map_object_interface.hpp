#pragma once

#include <ori/simcars/map/declarations.hpp>
#include <ori/simcars/map/map_interface.hpp>

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
    virtual IMap<T_id> const* get_map() const = 0;
};

template <typename T_id>
inline bool operator ==(IMapObject<T_id> const *lhs, IMapObject<T_id> const *rhs)
{
    return lhs->get_id() == rhs->get_id() && lhs->get_map() == rhs->get_map();
}

}
}
}
