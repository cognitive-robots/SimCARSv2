#pragma once

#include <ori/simcars/structures/read_only_array_interface.hpp>

#include <functional>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class AReadOnlyArray : public virtual IReadOnlyArray<T>
{
public:
    template <typename T_new>
    void map_to(std::function<T_new(const T&)> func, IArray<T_new>& new_array) const
    {
        size_t i;
        for (i = 0; i < this->count() || i < new_array.count(); ++i)
        {
            new_array[i] = func(this->operator [](i));
        }
    }
};

}
}
}
