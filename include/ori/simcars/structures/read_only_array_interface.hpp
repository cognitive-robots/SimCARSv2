#pragma once

#include <ori/simcars/structures/container_interface.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class IReadOnlyArray : public virtual IContainer<T>
{
public:
    virtual const T& operator [](size_t idx) const = 0;
};

}
}
}
