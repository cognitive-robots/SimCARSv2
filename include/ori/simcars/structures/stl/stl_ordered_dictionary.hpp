#pragma once

#include <ori/simcars/structures/dictionary_interface.hpp>
#include <ori/simcars/structures/stl/stl_stack_array.hpp>

#include <map>

namespace ori
{
namespace simcars
{
namespace structures
{
namespace stl
{

template <typename K, typename V, class K_compare = std::less<K>>
class STLOrderedDictionary : public virtual IDictionary<K, V>
{
protected:
    std::map<K, V, K_compare> data;

    mutable IStackArray<K> *keys_cache;
    mutable IStackArray<V> *values_cache;

public:
    STLOrderedDictionary() : keys_cache(nullptr), values_cache(nullptr) {}
    STLOrderedDictionary(STLOrderedDictionary<K, V, K_compare> const &stl_dictionary) : data(stl_dictionary.data) {}
    STLOrderedDictionary(IDictionary<K, V> const *dictionary)
    {
        IArray<K> const *keys = dictionary->get_keys();
        for (size_t i = 0; i < keys->count(); ++i)
        {
            K key = (*keys)[i];
            data[key] = (*dictionary)[key];
        }
    }

    ~STLOrderedDictionary() override
    {
        delete keys_cache;
        delete values_cache;
    }

    size_t count() const override
    {
        return data.size();
    }
    bool contains(K const &key) const override
    {
        return data.count(key) > 0;
    }

    V const& operator [](K const &key) const override
    {
        return data.at(key);
    }
    bool contains_value(V const &val) const override
    {
        for (auto const &entry : data)
        {
            if (entry.second == val)
            {
                return true;
            }
        }

        return false;
    }
    IArray<K> const* get_keys() const override
    {
        if (keys_cache)
        {
            return keys_cache;
        }
        else
        {
            keys_cache = new STLStackArray<K>(count());
            get_keys(keys_cache);
            return keys_cache;
        }
    }
    void get_keys(IStackArray<K> *keys) const override
    {
        size_t i = 0;
        for (auto const &entry : data)
        {
            if (i >= keys->count())
            {
                keys->push_back(entry.first);
                ++i;
            }
            else
            {
                (*keys)[i] = entry.first;
                ++i;
            }
        }
    }
    IArray<V> const* get_values() const override
    {
        if (values_cache)
        {
            return values_cache;
        }
        else
        {
            values_cache = new STLStackArray<V>(count());
            get_values(values_cache);
            return values_cache;
        }
    }
    void get_values(IStackArray<V> *values) const override
    {
        size_t i = 0;
        for (auto const &entry : data)
        {
            if (i >= values->count())
            {
                values->push_back(entry.second);
                ++i;
            }
            else
            {
                (*values)[i] = entry.second;
                ++i;
            }
        }
    }

    void update(K const &key, V const &val) override
    {
        data[key] = val;

        delete keys_cache;
        keys_cache = nullptr;

        delete values_cache;
        values_cache = nullptr;
    }
    void erase(K const &key) override
    {
        data.erase(key);

        delete keys_cache;
        keys_cache = nullptr;

        delete values_cache;
        values_cache = nullptr;
    }
};

}
}
}
}
