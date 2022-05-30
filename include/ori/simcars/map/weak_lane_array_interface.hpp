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
class IWeakLaneArray : public virtual structures::IArray<std::weak_ptr<const ILane<T_id>>>, public virtual ISoul<IWeakLaneArray<T_id>>
{
};

}
}
}
