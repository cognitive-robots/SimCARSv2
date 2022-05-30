#pragma once

#include <ori/simcars/structures/array_abstract.hpp>
#include <ori/simcars/structures/stl/stl_read_only_concat_array.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{
namespace stl
{

template <typename T>
class STLConcatArray : public virtual STLReadOnlyConcatArray<T>, public virtual AArray<T>
{
public:
    using structures::stl::STLReadOnlyConcatArray<T>::STLReadOnlyConcatArray;

    T& operator [](size_t idx) override
    {
        size_t i = 0;
        while (true)
        {
            if (idx < this->data[i]->count())
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
};

}
}
}
}
