#pragma once

#include <ori/simcars/structures/array_interface.hpp>

#include <vector>
#include <mutex>

namespace ori
{
namespace simcars
{
namespace structures
{
namespace stl
{

template <typename T>
class STLConcatArray : public virtual IArray<T>
{
protected:
    std::vector<IArray<T>*> data;

    mutable std::recursive_mutex data_mutex;

public:
    STLConcatArray(size_t size = 0) : data(size, nullptr) {}
    STLConcatArray(std::initializer_list<IArray<T> const*> init_list) : data(init_list) {}
    STLConcatArray(STLConcatArray<T> const &stl_concat_array) : data(stl_concat_array.data) {}

    ~STLConcatArray() override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        for (IArray<T> *array : data)
        {
            delete array;
        }
    }

    size_t count() const override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        size_t cumil_count = 0;

        size_t i;
        for (i = 0; i < data.size(); ++i)
        {
            size_t current_count = data[i]->count();
            cumil_count += current_count;
        }

        return cumil_count;
    }
    bool contains(T const &val) const override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        size_t i;
        for (i = 0; i < data.size(); ++i)
        {
            if (data[i]->contains(val))
            {
                return true;
            }
        }

        return false;
    }

    T const& operator [](size_t idx) const override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        size_t i = 0;
        while (true)
        {
            if (idx < data.at(i)->count())
            {
                return (*data[i])[idx];
            }
            else
            {
                idx -= data[i]->count();
                ++i;
            }
        }
    }

    T& operator [](size_t idx) override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        size_t i = 0;
        while (true)
        {
            if (idx < data.at(i)->count())
            {
                return (*data[i])[idx];
            }
            else
            {
                idx -= data[i]->count();
                ++i;
            }
        }
    }

    size_t array_count() const
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        return data.size();
    }
    bool contains_array(IArray<T> const *val) const
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        for (IArray<T> *data_val : data)
        {
            if (data_val == val)
            {
                return true;
            }
        }

        return false;
    }
    IArray<T>* const& get_array(size_t idx) const
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        return data.at(idx);
    }

    IArray<T>* & get_array(size_t idx)
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        return data.at(idx);
    }
};

}
}
}
}
