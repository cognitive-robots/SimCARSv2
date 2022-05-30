#pragma once

#include <ori/simcars/structures/array_abstract.hpp>
#include <ori/simcars/map/soul_abstract.hpp>
#include <ori/simcars/map/declarations.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template <typename T_id>
class AWeakTrafficLightArray : public IWeakTrafficLightArray<T_id>, public virtual structures::AArray<std::weak_ptr<const ITrafficLight<T_id>>>, public ASoul<IWeakTrafficLightArray<T_id>>
{
};

template<typename T_id>
inline bool operator ==(const std::weak_ptr<const ITrafficLight<T_id>>& lhs, const std::weak_ptr<const ITrafficLight<T_id>>& rhs)
{
    return lhs.lock() == rhs.lock();
}

}
}
}
