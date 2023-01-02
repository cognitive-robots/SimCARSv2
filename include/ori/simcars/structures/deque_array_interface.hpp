#pragma once

#include <ori/simcars/structures/deque_interface.hpp>
#include <ori/simcars/structures/array_interface.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class IDequeArray : public virtual IDeque<T>, public virtual IArray<T>
{
};

}
}
}
