#pragma once

#include <ori/simcars/structures/stack_array_interface.hpp>

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
class STLStackArray : public virtual IStackArray<T>
{
protected:
    std::vector<T> data;

public:
    STLStackArray() {}
    STLStackArray(size_t size, T const &default_value = T()) : data(size, default_value) {}
    STLStackArray(std::initializer_list<T> init_list) : data(init_list) {}
    STLStackArray(STLStackArray<T> const &stl_stack_array) : data(stl_stack_array.data) {}
    STLStackArray(IArray<T> const *array) : data(array->count())
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

    T const& peek_back() const override
    {
        return data.back();
    }

    T const& operator [](size_t idx) const override
    {
        return data.at(idx);
    }

    void push_back(T const &val) override
    {
        data.push_back(val);
    }
    void clear() override
    {
        data.clear();
    }
    void resize(size_t size) override
    {
        data.resize(size);
    }

    void erase_back() override
    {
        data.pop_back();
    }
    T pop_back() override
    {
        T const val = peek_back();
        data.pop_back();
        return val;
    }

    T& operator [](size_t idx) override
    {
        return data.at(idx);
    }
};

}
}
}
}
