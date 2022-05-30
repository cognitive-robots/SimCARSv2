#pragma once

#include <ori/simcars/temporal/typedefs.hpp>

#include <memory>

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

    virtual std::shared_ptr<const TState> get_state(Time timestamp) const = 0;
};

}
}
}
