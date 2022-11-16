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
protected:
    virtual bool is_equal(IValuelessEvent const *event) const = 0;

public:
    virtual ~IValuelessEvent() = default;

    virtual IValuelessEvent* valueless_shallow_copy() const = 0;

    virtual std::string get_variable_name() const = 0;
    virtual std::string get_value_as_string() const = 0;
    virtual temporal::Time get_time() const = 0;

    friend inline bool operator ==(IValuelessEvent const *lhs, IValuelessEvent const *rhs);
};

inline bool operator ==(IValuelessEvent const *lhs, IValuelessEvent const *rhs)
{
    return typeid(*lhs) == typeid(*rhs) && lhs->is_equal(rhs);
}

}
}
}
