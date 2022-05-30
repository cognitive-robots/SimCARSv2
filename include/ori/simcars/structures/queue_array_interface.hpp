#pragma once

#include <ori/simcars/structures/queue_interface.hpp>
#include <ori/simcars/structures/array_interface.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class IQueueArray : public virtual IQueue<T>, public virtual IArray<T>
{
};

}
}
}
