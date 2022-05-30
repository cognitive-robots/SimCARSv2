#pragma once

#include <ori/simcars/structures/read_only_array_abstract.hpp>

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
class STLReadOnlyConcatArray : public virtual AReadOnlyArray<T>
{
protected:
    std::vector<std::shared_ptr<const IReadOnlyArray<T>>> data;

public:
    STLReadOnlyConcatArray(size_t size = 0) : data(size) {}
    STLReadOnlyConcatArray(std::initializer_list<std::shared_ptr<const IArray<T>>> init_list) : data(init_list) {}
    STLReadOnlyConcatArray(const STLReadOnlyConcatArray<T>& stl_read_only_concat_array) : data(stl_read_only_concat_array.data) {}

    size_t count() const override
    {
        size_t cumil_count;

        size_t i;
        for (i = 0; i < data.size(); ++i)
        {
            cumil_count += data[i]->count();
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
            if (idx < data[i]->count())
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
        return data.size();
    }

    bool contains_array(std::shared_ptr<const IReadOnlyArray<T>> val) const
    {
        for (std::shared_ptr<const IReadOnlyArray<T>> data_val : data)
        {
            if (data_val == val)
            {
                return true;
            }
        }

        return false;
    }

    const std::shared_ptr<const IReadOnlyArray<T>>& get_array(size_t idx) const
    {
        return data[idx];
    }

    std::shared_ptr<const IReadOnlyArray<T>>& get_array(size_t idx)
    {
        return data[idx];
    }
};

}
}
}
}
