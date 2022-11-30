#pragma once

#include <ori/simcars/agent/constant_interface.hpp>
#include <ori/simcars/agent/event_abstract.hpp>
#include <ori/simcars/agent/basic_event.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class BasicEvent : public virtual AEvent<T>
{
    std::string const entity_name;
    std::string const parameter_name;
    T const value;
    temporal::Time const time;

public:
    BasicEvent(std::string const &entity_name,
               std::string const &parameter_name,
               T value, temporal::Time time) :
        entity_name(entity_name), parameter_name(parameter_name),
        value(value), time(time) {}
    BasicEvent(IConstant<T> const *parameter_value, temporal::Time time) :
        entity_name(parameter_value->get_entity_name()),
        parameter_name(parameter_value->get_parameter_name()),
        value(parameter_value->get_value()), time(time) {}

    IConstant<T>* constant_shallow_copy() const override
    {
        return this->event_shallow_copy();
    }

    IEvent<T>* event_shallow_copy() const override
    {
        return new BasicEvent<T>(entity_name, parameter_name, value, time);
    }

    T get_value() const override
    {
        return value;
    }

    std::string get_entity_name() const override
    {
        return entity_name;
    }

    std::string get_parameter_name() const override
    {
        return parameter_name;
    }

    temporal::Time get_time() const override
    {
        return time;
    }
};

}
}
}
