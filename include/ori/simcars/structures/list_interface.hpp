#pragma once

#include <ori/simcars/structures/container_interface.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class IList : public virtual IContainer<T>
{
public:
    virtual void push_back(const T& val) = 0;
    virtual void clear() = 0;
};

}
}
}
