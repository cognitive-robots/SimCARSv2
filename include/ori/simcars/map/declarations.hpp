#pragma once

#include <ori/simcars/structures/array_interface.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class IMap;

template <typename T_id>
class IMapObject;

template <typename T_id>
class ILane;

template <typename T_id>
using ILaneArray = structures::IArray<ILane<T_id> const*>;

template <typename T_id>
class ITrafficLight;

template <typename T_id>
using ITrafficLightArray = structures::IArray<ITrafficLight<T_id> const*>;

}
}
}
