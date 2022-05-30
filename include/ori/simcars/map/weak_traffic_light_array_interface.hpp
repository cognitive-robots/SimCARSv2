#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/map/soul_interface.hpp>
#include <ori/simcars/map/declarations.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class IWeakTrafficLightArray : public virtual structures::IArray<std::weak_ptr<const ITrafficLight<T_id>>>, public virtual ISoul<IWeakTrafficLightArray<T_id>>
{
};

}
}
}
