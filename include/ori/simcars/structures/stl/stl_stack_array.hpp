#pragma once

#include <ori/simcars/structures/stack_array_abstract.hpp>

#include <vector>

namespace ori
{
namespace simcars
{
namespace structures
{
namespace stl
{

template <typename T>
class STLStackArray : public virtual AStackArray<T>
{
protected:
    std::vector<T> data;

public:
    STLStackArray(size_t size = 0) : data(size) {}
    STLStackArray(std::initializer_list<T> init_list) : data(init_list) {}
    STLStackArray(const STLStackArray<T>& stl_stack_array) : data(stl_stack_array.data) {}

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

    const T& peek_back() const override
    {
        return data.back();
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

    void erase_back() override
    {
        data.pop_back();
    }
    T pop_back() override
    {
        const T val = peek_back();
        data.pop_back();
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
