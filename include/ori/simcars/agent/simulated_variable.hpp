#pragma once

#include <ori/simcars/structures/stl/stl_concat_array.hpp>
#include <ori/simcars/temporal/precedence_temporal_dictionary.hpp>
#include <ori/simcars/agent/simulated_valueless_variable_interface.hpp>
#include <ori/simcars/agent/constant_interface.hpp>
#include <ori/simcars/agent/entity_state_interface.hpp>
#include <ori/simcars/agent/simulation_scene_interface.hpp>
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
class SimulatedVariable : public virtual AVariable<T>, public virtual ISimulatedValuelessVariable
{
    IVariable<T> const *original_variable;
    ISimulationScene *simulation_scene;

    mutable temporal::Time simulation_start_time;
    temporal::Time simulation_end_time;

    mutable temporal::PrecedenceTemporalDictionary<IEvent<T>*> time_event_dict;

    void simulation_check(temporal::Time time) const
    {
        temporal::Time simulation_target_time = std::min(time, simulation_end_time);
        if (time_event_dict.count() == 0)
        {
            if (simulation_target_time >= simulation_start_time + simulation_scene->get_time_step())
            {
                simulation_scene->simulate(simulation_target_time);
            }
        }
        else if (time_event_dict.get_latest_timestamp() + simulation_scene->get_time_step() <= simulation_target_time)
        {
            simulation_scene->simulate(simulation_target_time);
        }
    }

public:
    SimulatedVariable(IVariable<T> const *original_variable,
                      ISimulationScene *simulation_scene,
                      temporal::Time simulation_start_time,
                      bool start_simulated,
                      temporal::Duration time_diff_threshold = temporal::Duration::max() / 2,
                      size_t max_cache_size = 10) :
        SimulatedVariable(original_variable, simulation_scene,
                          simulation_start_time,
                          original_variable->get_max_temporal_limit(),
                          start_simulated,
                          time_diff_threshold,
                          max_cache_size) {}
    SimulatedVariable(IVariable<T> const *original_variable,
                      ISimulationScene *simulation_scene,
                      temporal::Time simulation_start_time,
                      temporal::Time simulation_end_time,
                      bool start_simulated,
                      temporal::Duration time_diff_threshold = temporal::Duration::max() / 2,
                      size_t max_cache_size = 10) :
        original_variable(original_variable), simulation_scene(simulation_scene),
        simulation_end_time(simulation_end_time),
        time_event_dict(time_diff_threshold, max_cache_size)
    {
        if (simulation_start_time < original_variable->get_min_temporal_limit())
        {
            throw std::invalid_argument("Simulation start time is before earliest event for original variable");
        }

        if (simulation_start_time > original_variable->get_max_temporal_limit())
        {
            throw std::invalid_argument("Simulation start time is after latest event for original variable");
        }

        if (simulation_start_time > simulation_end_time)
        {
            throw std::invalid_argument("Simulation start time is after simulation end time");
        }

        if (start_simulated)
        {
            this->simulation_start_time = simulation_start_time;
        }
        else
        {
            this->simulation_start_time = simulation_end_time;
        }
    }

    ~SimulatedVariable()
    {
        structures::IArray<IEvent<T>*> const *events = time_event_dict.get_values();

        for (size_t i = 0; i < events->count(); ++i)
        {
            delete (*events)[i];
        }
    }

    IValuelessVariable* valueless_deep_copy() const override
    {
        return deep_copy();
    }

    /*
     * NOTE: This should not be called directly by external code, as the simulation scene will not have a pointer to the
     * resulting copy and thus any new simulation data will not be propogated to the copy.
     */
    IVariable<T>* deep_copy() const override
    {
        SimulatedVariable<T> *variable =
                new SimulatedVariable<T>(original_variable, simulation_scene, simulation_start_time,
                                         simulation_end_time, true, time_event_dict.get_time_diff_threshold(), time_event_dict.get_max_cache_size());
        size_t i;

        structures::IArray<temporal::Time> const *times = time_event_dict.get_keys();
        for(i = 0; i < times->count(); ++i)
        {
            variable->time_event_dict.update((*times)[i], time_event_dict[(*times)[i]]->event_shallow_copy());
        }

        return variable;
    }

    ISimulatedValuelessVariable* simulated_valueless_deep_copy() const override
    {
        return dynamic_cast<ISimulatedValuelessVariable*>(deep_copy());
    }

    std::string get_entity_name() const override
    {
        return original_variable->get_entity_name();
    }

    std::string get_variable_name() const override
    {
        return original_variable->get_variable_name();
    }

    IValuelessVariable::Type get_type() const override
    {
        return original_variable->get_type();
    }

    temporal::Time get_min_temporal_limit() const override
    {
        return original_variable->get_min_temporal_limit();
    }

    temporal::Time get_max_temporal_limit() const override
    {
        return simulation_end_time;
    }

    bool has_event(temporal::Time time) const override
    {
        return time_event_dict.contains(time) || (time <= simulation_start_time && original_variable->has_event(time));
    }

    T const& get_value(temporal::Time time) const override
    {
        simulation_check(time);

        if (time < simulation_start_time + simulation_scene->get_time_step())
        {
            return original_variable->get_value(time);
        }
        else
        {
            return time_event_dict[time]->get_value();
        }
    }

    structures::IArray<IEvent<T> const*>* get_events(
            temporal::Time time_window_start,
            temporal::Time time_window_end) const override
    {
        simulation_check(time_window_end);

        structures::stl::STLConcatArray<IEvent<T> const*> *events =
                new structures::stl::STLConcatArray<IEvent<T> const*>(2);

        events->get_array(0) = original_variable->get_events(time_window_start, time_window_end);

        structures::IArray<IEvent<T>*> const *unfiltered_simulated_events =
                time_event_dict.get_values();
        structures::IStackArray<IEvent<T> const*> *filtered_simulated_events =
                new structures::stl::STLStackArray<IEvent<T> const*>;

        for (size_t i = 0; i < unfiltered_simulated_events->count(); ++i)
        {
            if ((*unfiltered_simulated_events)[i]->get_time() >= time_window_start
                    && (*unfiltered_simulated_events)[i]->get_time() <= time_window_end)
            {
                filtered_simulated_events->push_back((*unfiltered_simulated_events)[i]);
            }
        }

        events->get_array(1) = filtered_simulated_events;

        return events;
    }

    IEvent<T> const* get_event(temporal::Time time, bool exact) const override
    {
        simulation_check(time);

        if (time < simulation_start_time + simulation_scene->get_time_step())
        {
            return original_variable->get_event(time, exact);
        }
        else
        {
            IEvent<T> const *prospective_event = time_event_dict[time];
            if (!exact || prospective_event->get_time() == time)
            {
                return prospective_event;
            }
            else
            {
                throw std::out_of_range("No event at specified time");
            }
        }
    }

    void set_value(temporal::Time time, T const &value) override
    {
        if (time > simulation_start_time
                && time <= simulation_end_time)
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

            time_event_dict.update(time,
                                   new BasicEvent(this->get_entity_name(),
                                                  this->get_parameter_name(),
                                                  value, time));
        }
    }

    structures::IArray<IEvent<T>*>* get_mutable_events(
            temporal::Time time_window_start,
            temporal::Time time_window_end) override
    {
        simulation_check(time_window_end);

        structures::IArray<IEvent<T>*> const *unfiltered_simulated_events =
                time_event_dict.get_values();
        structures::IStackArray<IEvent<T>*> *filtered_simulated_events =
                new structures::stl::STLStackArray<IEvent<T>*>;

        for (size_t i = 0; i < unfiltered_simulated_events->count(); ++i)
        {
            if ((*unfiltered_simulated_events)[i]->get_time() >= time_window_start
                    && (*unfiltered_simulated_events)[i]->get_time() <= time_window_end)
            {
                filtered_simulated_events->push_back((*unfiltered_simulated_events)[i]);
            }
        }

        return filtered_simulated_events;
    }

    IEvent<T>* get_mutable_event(temporal::Time time, bool exact) override
    {
        simulation_check(time);

        if (time >= simulation_start_time + simulation_scene->get_time_step())
        {
            IEvent<T> *prospective_event = time_event_dict[time];
            if (!exact || prospective_event->get_time() == time)
            {
                return prospective_event;
            }
            else
            {
                throw std::out_of_range("No event at specified time");
            }
        }
        else
        {
            throw std::out_of_range("Specified time outside of simulation window");
        }
    }

    bool simulation_update(temporal::Time time, IEntityState const *state) const override
    {
        if (time <= simulation_start_time
                || time > simulation_end_time)
        {
            return false;
        }
        else
        {
            try
            {
                IValuelessConstant const *valueless_parameter_value =
                        state->get_parameter_value(this->get_full_name());

                IConstant<T> const *parameter_value =
                        dynamic_cast<IConstant<T> const*>(valueless_parameter_value);
                IEvent<T> *update_event = new BasicEvent(parameter_value, time);

                time_event_dict.update(update_event->get_time(), update_event);
            }
            catch (std::out_of_range)
            {
                return false;
            }

            return true;
        }
    }

    void begin_simulation(temporal::Time simulation_start_time) const override
    {
        if (this->simulation_start_time == this->simulation_end_time &&
                simulation_start_time <= original_variable->get_max_temporal_limit())
        {
            this->simulation_start_time = simulation_start_time;
        }
    }
};

}
}
}
