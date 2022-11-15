#pragma once

#include <ori/simcars/structures/container_interface.hpp>
#include <ori/simcars/structures/stack_array_interface.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename K, typename V>
class IDictionary : public virtual IContainer<K>
{
public:
    virtual V const& operator [](K const &key) const = 0;
    virtual bool contains_value(V const &val) const = 0;
    virtual IArray<K> const* get_keys() const = 0;
    virtual void get_keys(IStackArray<K> *keys) const = 0;
    virtual IArray<V> const* get_values() const = 0;
    virtual void get_values(IStackArray<V> *values) const = 0;

    virtual void update(K const &key, V const &val) = 0;
    virtual void erase(K const &key) = 0;
};

}
}
}
