#pragma once

#include <ori/simcars/structures/container_interface.hpp>
#include <ori/simcars/structures/array_interface.hpp>

#include <memory>

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
    virtual std::shared_ptr<const IArray<T>> get_array() const = 0;

    virtual void union_with(std::shared_ptr<const ISet<T>> set) = 0;
    virtual void intersect_with(std::shared_ptr<const ISet<T>> set) = 0;
    virtual void difference_with(std::shared_ptr<const ISet<T>> set) = 0;
    virtual void insert(const T& val) = 0;
    virtual void erase(const T& val) = 0;
};

}
}
}
