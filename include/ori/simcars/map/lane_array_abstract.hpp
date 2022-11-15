#pragma once

#include <ori/simcars/map/lane_array_interface.hpp>
#include <ori/simcars/map/soul_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class ALaneArray : public virtual ILaneArray<T_id>, public virtual ASoul<ILaneArray<T_id>>
{
};

}
}
}
