#pragma once

#include <ori/simcars/structures/array_interface.hpp>

#include <vector>
#include <memory>

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
public:
protected:
    std::vector<std::shared_ptr<IArray<T>>> data;

public:
    STLConcatArray(size_t size = 0) : data(size) {}
    STLConcatArray(std::initializer_list<std::shared_ptr<const IArray<T>>> init_list) : data(init_list) {}
    STLConcatArray(const STLConcatArray<T>& stl_concat_array) : data(stl_concat_array.data) {}

    size_t count() const override
    {
        size_t cumil_count = 0;

        size_t i;
        for (i = 0; i < data.size(); ++i)
        {
            size_t current_count = data[i]->count();
            cumil_count += current_count;
        }

        return cumil_count;
    }
    bool contains(const T& val) const override
    {
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

    const T& operator [](size_t idx) const override
    {
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
        size_t i = 0;
        while (true)
        {
            if (idx < this->data.at(i)->count())
            {
                return (*this->data[i])[idx];
            }
            else
            {
                idx -= this->data[i]->count();
                ++i;
            }
        }
    }

    size_t array_count() const
    {
        return data.size();
    }

    bool contains_array(std::shared_ptr<const IArray<T>> val) const
    {
        for (std::shared_ptr<const IArray<T>> data_val : data)
        {
            if (data_val == val)
            {
                return true;
            }
        }

        return false;
    }

    const std::shared_ptr<IArray<T>>& get_array(size_t idx) const
    {
        return data.at(idx);
    }

    std::shared_ptr<IArray<T>>& get_array(size_t idx)
    {
        return data.at(idx);
    }
};

}
}
}
}
