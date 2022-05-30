#pragma once

#include <ori/simcars/map/soul_abstract.hpp>
#include <ori/simcars/map/declarations.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class AWeakLaneArray : public virtual IWeakLaneArray<T_id>, public ASoul<IWeakLaneArray<T_id>>
{
};

template<typename T_id>
inline bool operator ==(const std::weak_ptr<const ILane<T_id>>& lhs, const std::weak_ptr<const ILane<T_id>>& rhs)
{
    return lhs.lock() == rhs.lock();
}

}
}
}
