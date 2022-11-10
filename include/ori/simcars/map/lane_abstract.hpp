#pragma once

#include <ori/simcars/map/soul_abstract.hpp>
#include <ori/simcars/map/lane_interface.hpp>
#include <ori/simcars/map/map_object.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class ALane : public ILane<T_id>, public MapObject<T_id>, public virtual ASoul<ILane<T_id>>
{
public:
    ALane(T_id const &id, IMap<T_id> const *map) : MapObject<T_id>(id, map) {}
};

}
}
}
