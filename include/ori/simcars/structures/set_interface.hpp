#pragma once

#include <ori/simcars/structures/container_interface.hpp>
#include <ori/simcars/structures/array_interface.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class ISet : public virtual IContainer<T>
{
public:
    virtual IArray<T> const* get_array() const = 0;

    virtual void union_with(ISet<T> const *set) = 0;
    virtual void intersect_with(ISet<T> const *set) = 0;
    virtual void difference_with(ISet<T> const *set) = 0;
    virtual void insert(T const &val) = 0;
    virtual void erase(T const &val) = 0;
};

}
}
}
