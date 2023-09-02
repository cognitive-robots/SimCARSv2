#pragma once

#include <ori/simcars/structures/list_interface.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class IRAModList : public virtual IList<T>
{
public:
    virtual void push_at(size_t idx, T const &val) = 0;
    virtual void erase_at(size_t idx) = 0;
};

}
}
}
