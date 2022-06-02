#pragma once

#include <ori/simcars/temporal/typedefs.hpp>

#include <string>
#include <memory>

namespace ori
{
namespace simcars
{
namespace agent
{

class IValuelessEvent
{
protected:
    virtual bool is_equal(std::shared_ptr<const IValuelessEvent> event) const = 0;

public:
    virtual ~IValuelessEvent() = default;

    virtual std::shared_ptr<IValuelessEvent> valueless_shallow_copy() const = 0;

    virtual std::string get_variable_name() const = 0;
    virtual std::string get_value_as_string() const = 0;
    virtual temporal::Time get_time() const = 0;

    friend inline bool operator ==(std::shared_ptr<const IValuelessEvent> lhs, std::shared_ptr<const IValuelessEvent> rhs);
};

inline bool operator ==(std::shared_ptr<const IValuelessEvent> lhs, std::shared_ptr<const IValuelessEvent> rhs)
{
    return typeid(*lhs) == typeid(*rhs) && lhs->is_equal(rhs);
}

}
}
}
