#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/map/soul_interface.hpp>
#include <ori/simcars/map/lane_interface.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class ILaneArray : public virtual structures::IArray<ILane<T_id> const*>, public virtual ISoul<ILaneArray<T_id>>
{
};

}
}
}
