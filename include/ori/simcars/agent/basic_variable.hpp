#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/temporal/temporal_rounding_dictionary.hpp>
#include <ori/simcars/agent/variable_abstract.hpp>
#include <ori/simcars/agent/basic_event.hpp>

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
    std::string const entity_name;
    std::string const variable_name;

    IValuelessVariable::Type const type;

    temporal::TemporalRoundingDictionary<IEvent<T>*> time_event_dict;

public:
    BasicVariable(std::string const &entity_name, std::string const &parameter_name, IValuelessVariable::Type type,
             temporal::Duration time_step) : entity_name(entity_name), variable_name(parameter_name), type(type),
        time_event_dict(time_step, nullptr) {}

    ~BasicVariable()
    {
        structures::IArray<IEvent<T>*> const *events = time_event_dict.get_values();

        for (size_t i = 0; i < events->count(); ++i)
        {
            delete (*events)[i];
        }
    }

    IValuelessVariable* valueless_deep_copy() const override
    {
        return variable_deep_copy();
    }

    IVariable<T>* variable_deep_copy() const override
    {
        BasicVariable<T> *variable =
                new BasicVariable<T>(
                    entity_name,
                    variable_name,
                    type,
                    time_event_dict.get_time_window_step());

        structures::IArray<temporal::Time> const *times = time_event_dict.get_keys();
        for(size_t i = 0; i < times->count(); ++i)
        {
            IEvent<T> *event = time_event_dict[(*times)[i]];
            if (event != nullptr)
            {
                variable->time_event_dict.update((*times)[i], event->event_shallow_copy());
            }
        }

        variable->propogate_events_forward(this->get_max_temporal_limit());

        return variable;
    }

    std::string get_entity_name() const override
    {
        return entity_name;
    }

    std::string get_variable_name() const override
    {
        return variable_name;
    }

    IValuelessVariable::Type get_type() const override
    {
        return type;
    }

    temporal::Time get_min_temporal_limit() const override
    {
        return time_event_dict.get_earliest_timestamp();
    }

    temporal::Time get_last_event_time() const override
    {
        return time_event_dict.get_latest_timestamp();
    }

    temporal::Time get_max_temporal_limit() const override
    {
        return time_event_dict.get_latest_timestamp();
    }

    bool has_event(temporal::Time time) const override
    {
        return time_event_dict.contains(time);
    }

    bool get_value(temporal::Time time, T &value) const override
    {
        if (time_event_dict.contains(time))
        {
            value = time_event_dict[time]->get_value();
            return true;
        }
        else
        {
            return false;
        }
    }

    structures::IArray<IEvent<T> const*>* get_events(
            temporal::Time time_window_start,
            temporal::Time time_window_end) const override
    {
        if (time_event_dict.count() == 0 ||
                time_window_end < this->get_min_temporal_limit() ||
                time_window_start > this->get_max_temporal_limit())
        {
            return new structures::stl::STLStackArray<IEvent<T> const*>;
        }

        structures::IArray<IEvent<T>*> const *unfiltered_events =
                time_event_dict.get_values();
        structures::IStackArray<IEvent<T> const*> *filtered_events =
                new structures::stl::STLStackArray<IEvent<T> const*>;

        for (size_t i = 0; i < unfiltered_events->count(); ++i)
        {
            if ((*unfiltered_events)[i] != nullptr &&
                    (*unfiltered_events)[i]->get_time() >= time_window_start &&
                    (*unfiltered_events)[i]->get_time() <= time_window_end)
            {
                filtered_events->push_back((*unfiltered_events)[i]);
            }
        }

        return filtered_events;
    }

    IEvent<T> const* get_event(temporal::Time time, bool exact) const override
    {
        IEvent<T> const *prospective_event = time_event_dict[time];
        if (!exact || prospective_event->get_time() == time)
        {
            return prospective_event;
        }
        else
        {
            return nullptr;
        }
    }

    bool remove_value(temporal::Time time) override
    {
        if (time_event_dict.contains(time, true))
        {
            IEvent<T> *event = time_event_dict[time];
            time_event_dict.erase(time);
            delete event;
            return true;
        }
        else
        {
            return false;
        }
    }

    void propogate_events_forward() override
    {
        time_event_dict.propogate_values_forward();
    }
    void propogate_events_forward(temporal::Time time_window_end) override
    {
        time_event_dict.propogate_values_forward(time_window_end);
    }

    void set_value(temporal::Time time, T const &value) override
    {
        if (time_event_dict.contains(time))
        {
            IEvent<T> *other_event = time_event_dict[time];
            if (time == other_event->get_time())
            {
                other_event->set_value(value);
                return;
            }
        }

        time_event_dict.update(time, new BasicEvent(this->get_entity_name(),
                                                    this->get_parameter_name(),
                                                    value, time));
    }

    structures::IArray<IEvent<T>*>* get_mutable_events(
            temporal::Time time_window_start,
            temporal::Time time_window_end) override
    {
        if (time_event_dict.count() == 0 ||
                time_window_end < this->get_min_temporal_limit() ||
                time_window_start > this->get_max_temporal_limit())
        {
            return new structures::stl::STLStackArray<IEvent<T>*>;
        }

        structures::IArray<IEvent<T>*> const *unfiltered_events =
                time_event_dict.get_values();
        structures::IStackArray<IEvent<T>*> *filtered_events =
                new structures::stl::STLStackArray<IEvent<T>*>;

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

    IEvent<T>* get_mutable_event(temporal::Time time, bool exact) override
    {
        IEvent<T> *prospective_event = time_event_dict[time];
        if (!exact || prospective_event->get_time() == time)
        {
            return prospective_event;
        }
        else
        {
            return nullptr;
        }
    }
};

}
}
}
