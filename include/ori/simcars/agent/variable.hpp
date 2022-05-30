#pragma once

#include <ori/simcars/temporal/precedence_temporal_dictionary.hpp>
#include <ori/simcars/agent/variable_abstract.hpp>

#include <stdexcept>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class Variable : public AVariable<T>
{
    const std::string entity_name;
    const std::string parameter_name;

    const IValuelessVariable::Type type;

    temporal::PrecedenceTemporalDictionary<std::shared_ptr<const IEvent<T>>> time_event_dict;

public:
    Variable(const std::string& entity_name, const std::string& parameter_name, IValuelessVariable::Type type,
             size_t max_cache_size = 10) : entity_name(entity_name), parameter_name(parameter_name), type(type),
        time_event_dict(max_cache_size) {}

    std::shared_ptr<IValuelessVariable> valueless_deep_copy() const override
    {
        return deep_copy();
    }

    std::shared_ptr<IVariable<T>> deep_copy() const override
    {
        std::shared_ptr<Variable<T>> variable(new Variable<T>(entity_name, parameter_name, type, time_event_dict.get_max_cache_size()));

        size_t i;

        std::shared_ptr<const structures::IArray<temporal::Time>> times = time_event_dict.get_keys();
        for(i = 0; i < times->count(); ++i)
        {
            variable->time_event_dict.update((*times)[i], time_event_dict[(*times)[i]]->shallow_copy());
        }

        return variable;
    }

    std::string get_entity_name() const override
    {
        return entity_name;
    }

    std::string get_parameter_name() const override
    {
        return parameter_name;
    }

    IValuelessVariable::Type get_type() const override
    {
        return type;
    }

    T get_value(temporal::Time time) const override
    {
        return time_event_dict[time]->get_value();
    }

    std::shared_ptr<structures::IArray<std::shared_ptr<const IEvent<T>>>> get_events() const override
    {
        return time_event_dict.get_values();
    }

    std::shared_ptr<const IEvent<T>> get_event(temporal::Time time) const override
    {
        std::shared_ptr<const IEvent<T>> prospective_event = time_event_dict[time];
        if (prospective_event->get_time() == time)
        {
            return prospective_event;
        }
        else
        {
            throw std::out_of_range("No event at specified time");
        }
    }

    void add_event(std::shared_ptr<const IEvent<T>> event) override
    {
        time_event_dict.update(event->get_time(), event);
    }

    bool remove_event(std::shared_ptr<const IEvent<T>> event) override
    {
        try
        {
            if (time_event_dict[event->get_time()] == event)
            {
                time_event_dict.erase(event->get_time());
                return true;
            }
            else
            {
                return false;
            }
        }
        catch (std::out_of_range)
        {
            return false;
        }
    }
};

}
}
}
