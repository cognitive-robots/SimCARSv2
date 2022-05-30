#pragma once

#include <ori/simcars/structures/read_only_array_interface.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class IArray : public virtual IReadOnlyArray<T>
{
public:
    const T& operator [](size_t idx) const override = 0;
    virtual T& operator [](size_t idx) = 0;
};

}
}
}
