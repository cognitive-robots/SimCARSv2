#pragma once

#include <ori/simcars/structures/deque_array_interface.hpp>

#include <deque>
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
class STLDequeArray : public virtual IDequeArray<T>
{
protected:
    std::deque<T> data;

    mutable std::recursive_mutex data_mutex;

public:
    STLDequeArray() {}
    STLDequeArray(size_t size, T const &default_value = T()) : data(size, default_value) {}
    STLDequeArray(std::initializer_list<T> init_list) : data(init_list) {}
    STLDequeArray(STLDequeArray<T> const &stl_queue_array) : data(stl_queue_array.data) {}
    STLDequeArray(IArray<T> const *array) : data(array->count())
    {
        for (size_t i = 0; i < array->count(); ++i)
        {
            data[i] = (*array)[i];
        }
    }

    size_t count() const override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        return data.size();
    }
    bool contains(T const &val) const override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        for (T const &data_val : data)
        {
            if (data_val == val)
            {
                return true;
            }
        }
        return false;
    }

    T const& peek_front() const override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        return data.front();
    }

    T const& peek_back() const override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        return data.back();
    }

    T const& operator [](size_t idx) const override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        return data[idx];
    }

    void push_back(T const &val) override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        data.push_back(val);
    }
    void clear() override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        data.clear();
    }
    void resize(size_t size) override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        data.resize(size);
    }

    void erase_front() override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        data.pop_front();
    }
    T pop_front() override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        T const val = peek_front();
        data.pop_front();
        return val;
    }

    void erase_back() override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        data.pop_back();
    }
    T pop_back() override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        T const val = peek_back();
        data.pop_back();
        return val;
    }

    T& operator [](size_t idx) override
    {
        std::lock_guard<std::recursive_mutex> data_guard(data_mutex);

        return data[idx];
    }
};

}
}
}
}
