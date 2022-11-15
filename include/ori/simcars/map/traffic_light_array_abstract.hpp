#pragma once

#include <ori/simcars/map/traffic_light_array_interface.hpp>
#include <ori/simcars/map/soul_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class ATrafficLightArray : public virtual ITrafficLightArray<T_id>, public virtual ASoul<ITrafficLightArray<T_id>>
{
};

}
}
}
