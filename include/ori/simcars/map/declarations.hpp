#pragma once

#include <ori/simcars/structures/array_interface.hpp>

#include <memory>

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
using ILaneArray = structures::IArray<std::shared_ptr<const ILane<T_id>>>;

template <typename T_id>
class IWeakLaneArray;

template <typename T_id>
class ITrafficLight;

template <typename T_id>
using ITrafficLightArray = structures::IArray<std::shared_ptr<const ITrafficLight<T_id>>>;

template <typename T_id>
class IWeakTrafficLightArray;

}
}
}
