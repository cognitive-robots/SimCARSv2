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
    STLQueueArray(const STLQueueArray<T>& stl_queue_array) : data(stl_queue_array.data) {}

    size_t count() const override
    {
        return data.size();
    }
    bool contains(const T& val) const override
    {
        for (const T& data_val : data)
        {
            if (data_val == val)
            {
                return true;
            }
        }
        return false;
    }

    const T& peek_front() const override
    {
        return data.front();
    }

    const T& operator [](size_t idx) const override
    {
        return data[idx];
    }

    void push_back(const T& val) override
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
        const T val = peek_front();
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
