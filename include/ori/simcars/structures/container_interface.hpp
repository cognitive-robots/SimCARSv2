#pragma once

#include <cstddef>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class IContainer
{
public:
    virtual ~IContainer() = default;

    virtual size_t count() const = 0;
    virtual bool contains(const T& val) const = 0;
};

}
}
}
