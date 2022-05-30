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

template <typename K, typename V>
class IDictionary : public virtual IContainer<K>
{
public:
    virtual const V& operator [](const K& key) const = 0;
    virtual const bool contains_value(const V& val) const = 0;
    virtual std::shared_ptr<IArray<K>> get_keys() const = 0;
    virtual std::shared_ptr<IArray<V>> get_values() const = 0;

    virtual void update(const K& key, const V& val) = 0;
    virtual void erase(const K& key) = 0;
};

}
}
}
