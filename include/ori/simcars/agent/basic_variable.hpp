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
class BasicVariable : public AVariable<T>
{
    const std::string entity_name;
    const std::string parameter_name;

    const IValuelessVariable::Type type;

    temporal::PrecedenceTemporalDictionary<std::shared_ptr<const IEvent<T>>> time_event_dict;

public:
    BasicVariable(const std::string& entity_name, const std::string& parameter_name, IValuelessVariable::Type type,
             temporal::Duration time_diff_threshold = temporal::Duration::max() / 2, size_t max_cache_size = 10) : entity_name(entity_name), parameter_name(parameter_name), type(type),
        time_event_dict(time_diff_threshold, max_cache_size) {}

    std::shared_ptr<IValuelessVariable> valueless_deep_copy() const override
    {
        return deep_copy();
    }

    std::shared_ptr<IVariable<T>> deep_copy() const override
    {
        std::shared_ptr<BasicVariable<T>> variable(
                    new BasicVariable<T>(
                        entity_name,
                        parameter_name,
                        type,
                        time_event_dict.get_time_diff_threshold(),
                        time_event_dict.get_max_cache_size()));

        std::shared_ptr<const structures::IArray<temporal::Time>> times = time_event_dict.get_keys();
        for(size_t i = 0; i < times->count(); ++i)
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

    temporal::Time get_min_temporal_limit() const override
    {
        return time_event_dict.get_earliest_timestamp();
    }

    temporal::Time get_max_temporal_limit() const override
    {
        return time_event_dict.get_latest_timestamp();
    }

    T get_value(temporal::Time time) const override
    {
        return time_event_dict[time]->get_value();
    }

    std::shared_ptr<structures::IArray<std::shared_ptr<const IEvent<T>>>> get_events(
            temporal::Time time_window_start,
            temporal::Time time_window_end) const override
    {
        std::shared_ptr<const structures::IArray<std::shared_ptr<const IEvent<T>>>> unfiltered_events =
                time_event_dict.get_values();
        std::shared_ptr<structures::IStackArray<std::shared_ptr<const IEvent<T>>>> filtered_events(
                new structures::stl::STLStackArray<std::shared_ptr<const IEvent<T>>>());

        for (size_t i = 0; i < unfiltered_events->count(); ++i)
        {
            if ((*unfiltered_events)[i]->get_time() >= time_window_start
                    && (*unfiltered_events)[i]->get_time() <= time_window_end)
            {
                filtered_events->push_back((*unfiltered_events)[i]);
            }
        }

        return filtered_events;
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

    bool add_event(std::shared_ptr<const IEvent<T>> event) override
    {
        time_event_dict.update(event->get_time(), event);

        return true;
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
