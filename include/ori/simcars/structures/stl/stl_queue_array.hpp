#pragma once

#include <ori/simcars/structures/queue_array_interface.hpp>

#include <deque>

namespace ori
{
namespace simcars
{
namespace structures
{
namespace stl
{

template <typename T>
class STLQueueArray : public virtual IQueueArray<T>
{
protected:
    std::deque<T> data;

public:
    STLQueueArray() {}
    STLQueueArray(std::initializer_list<T> init_list) : data(init_list) {}
    STLQueueArray(STLQueueArray<T> const &stl_queue_array) : data(stl_queue_array.data) {}
    STLQueueArray(IArray<T> const *array) : data(array->count())
    {
        for (size_t i = 0; i < array->count(); ++i)
        {
            data[i] = (*array)[i];
        }
    }

    size_t count() const override
    {
        return data.size();
    }
    bool contains(T const &val) const override
    {
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
        return data.front();
    }

    T const& operator [](size_t idx) const override
    {
        return data[idx];
    }

    void push_back(T const &val) override
    {
        data.push_back(val);
    }
    void clear() override
    {
        data.clear();
    }

    void erase_front() override
    {
        data.pop_front();
    }
    T pop_front() override
    {
        T const val = peek_front();
        data.pop_front();
        return val;
    }

    T& operator [](size_t idx) override
    {
        return data[idx];
    }
};

}
}
}
}
