#pragma once

#include <ori/simcars/structures/container_interface.hpp>

#include <functional>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class IArray : public virtual IContainer<T>
{
public:
    virtual T const& operator [](size_t idx) const = 0;
    virtual T & operator [](size_t idx) = 0;
};

template<typename T_old, typename T_new>
void cast_array(IArray<T_old> const &old_array, IArray<T_new> &new_array)
{
    size_t i;
    for (i = 0; i < old_array.count() || i < new_array.count(); ++i)
    {
        new_array[i] = static_cast<T_new>(old_array[i]);
    }
}

template<typename T_old, typename T_new>
void map_array(IArray<T_old> const &old_array, IArray<T_new> &new_array,
               std::function<T_new(T_old const&)> func)
{
    size_t i;
    for (i = 0; i < old_array.count() || i < new_array.count(); ++i)
    {
        new_array[i] = func(old_array[i]);
    }
}

}
}
}
