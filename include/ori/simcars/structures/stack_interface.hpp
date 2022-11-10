#pragma once

#include <ori/simcars/structures/list_interface.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class IStack : public virtual IList<T>
{
public:
    virtual T const& peek_back() const = 0;

    virtual void erase_back() = 0;
    virtual T pop_back() = 0;
};

}
}
}
