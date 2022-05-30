#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/structures/read_only_array_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class AArray : public virtual AReadOnlyArray<T>, public virtual IArray<T>
{
public:
    template <typename T_old>
    void map_from(std::function<T(const T_old&)> func, const IArray<T_old>& old_array)
    {
        size_t i;
        for (i = 0; i < this->count() || i < old_array.count(); ++i)
        {
            this->operator [](i) = func(old_array[i]);
        }
    }

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
