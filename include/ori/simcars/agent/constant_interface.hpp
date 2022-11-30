#pragma once

#include <ori/simcars/agent/valueless_constant_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class IConstant : public virtual IValuelessConstant
{
public:
    virtual IConstant<T>* constant_shallow_copy() const = 0;

    virtual T get_value() const = 0;
};

}
}
}
