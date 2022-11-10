#pragma once

#include <ori/simcars/temporal/typedefs.hpp>

namespace ori
{
namespace simcars
{
namespace temporal
{

template <typename T>
class IStateful
{
public:
    typedef T::State TState;

    virtual ~IStateful() = default;

    virtual TState const* get_state(Time timestamp) const = 0;
};

}
}
}
