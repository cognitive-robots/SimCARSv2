#pragma once

#include <ori/simcars/structures/stack_array_interface.hpp>
#include <ori/simcars/structures/array_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class AStackArray : public virtual IStackArray<T>, public virtual AArray<T>
{
};

}
}
}
