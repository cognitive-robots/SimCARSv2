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
    const std::string variable_name;
    const T value;
    const temporal::Time time;

public:
    BasicEvent(const std::string& variable_name, T value, temporal::Time time) :
        variable_name(variable_name), value(value), time(time) {}
    BasicEvent(std::shared_ptr<const IConstant<T>> parameter_value, temporal::Time time) :
        variable_name(parameter_value->get_full_name()), value(parameter_value->get_value()), time(time) {}

    std::shared_ptr<IValuelessEvent> valueless_shallow_copy() const override
    {
        return shallow_copy();
    }

    std::shared_ptr<IEvent<T>> shallow_copy() const override
    {
        return std::shared_ptr<BasicEvent<T>>(new BasicEvent<T>(variable_name, value, time));
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
