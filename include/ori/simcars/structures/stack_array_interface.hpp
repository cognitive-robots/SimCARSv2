#pragma once

#include <ori/simcars/structures/stack_interface.hpp>
#include <ori/simcars/structures/array_interface.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class IStackArray : public virtual IStack<T>, public virtual IArray<T>
{
};

}
}
}
