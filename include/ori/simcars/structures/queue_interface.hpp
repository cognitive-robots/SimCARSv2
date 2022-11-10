#pragma once

#include <ori/simcars/structures/list_interface.hpp>

namespace ori
{
namespace simcars
{
namespace structures
{

template <typename T>
class IQueue : public virtual IList<T>
{
public:
    virtual T const& peek_front() const = 0;

    virtual void erase_front() = 0;
    virtual T pop_front() = 0;
};

}
}
}
