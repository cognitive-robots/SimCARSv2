#pragma once

#include <ori/simcars/temporal/stateful_interface.hpp>

namespace ori
{
namespace simcars
{
namespace temporal
{

template <typename T>
class ILiving : public IStateful<T>
{
public:
    virtual Time get_birth() const = 0;
    virtual Time get_death() const = 0;
};

}
}
}
