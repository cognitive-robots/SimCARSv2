#pragma once

#include <ori/simcars/structures/stack_interface.hpp>
#include <ori/simcars/structures/queue_interface.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class IDeque : public virtual IStack<T>, public virtual IQueue<T>
{
};

}
}
}
