#pragma once

#include <ori/simcars/temporal/typedefs.hpp>

#include <string>

namespace ori
{
namespace simcars
{
namespace agent
{

class IValuelessEvent
{
public:
    virtual ~IValuelessEvent() = default;

    virtual IValuelessEvent* valueless_shallow_copy() const = 0;

    virtual std::string get_variable_name() const = 0;
    virtual std::string get_value_as_string() const = 0;
    virtual temporal::Time get_time() const = 0;
};

}
}
}
