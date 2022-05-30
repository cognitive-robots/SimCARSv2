#pragma once

#include <ori/simcars/structures/queue_array_interface.hpp>
#include <ori/simcars/structures/array_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class AQueueArray : public virtual IQueueArray<T>, public virtual AArray<T>
{
};

}
}
}
