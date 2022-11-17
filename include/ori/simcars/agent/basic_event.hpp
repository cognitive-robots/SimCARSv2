#pragma once

#include <ori/simcars/agent/constant_interface.hpp>
#include <ori/simcars/agent/event_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class BasicEvent : public AEvent<T>
{
    std::string const variable_name;
    T const value;
    temporal::Time const time;

public:
    BasicEvent(std::string const &variable_name, T value, temporal::Time time) :
        variable_name(variable_name), value(value), time(time) {}
    BasicEvent(IConstant<T> const *parameter_value, temporal::Time time) :
        variable_name(parameter_value->get_full_name()), value(parameter_value->get_value()), time(time) {}

    IValuelessEvent* valueless_shallow_copy() const override
    {
        return shallow_copy();
    }

    IEvent<T>* shallow_copy() const override
    {
        return new BasicEvent<T>(variable_name, value, time);
    }

    std::string get_variable_name() const override
    {
        return variable_name;
    }

    T get_value() const override
    {
        return value;
    }

    temporal::Time get_time() const override
    {
        return time;
    }
};

}
}
}
