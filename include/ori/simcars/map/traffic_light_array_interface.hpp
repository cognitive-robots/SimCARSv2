#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/map/traffic_light_interface.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class ITrafficLightArray : public virtual structures::IArray<ITrafficLight<T_id> const*>, public virtual ISoul<ITrafficLightArray<T_id>>
{
};

}
}
}
